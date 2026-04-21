/*
 * serial_driver.h
 *
 * USART0 transport and TPacket framing.
 * USE_BAREMETAL_SERIAL: 0 = Arduino Serial library, 1 = ISR-driven USART0.
 */

#pragma once

#include "packets.h"
#include <avr/interrupt.h>
#include <string.h>

#define USE_BAREMETAL_SERIAL 1

#if USE_BAREMETAL_SERIAL

// Power-of-2 sizes allow fast wrap-around with bitwise AND.
#define TX_BUFFER_SIZE 128
#define TX_BUFFER_MASK (TX_BUFFER_SIZE - 1)
#define RX_BUFFER_SIZE 256
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)

volatile uint8_t tx_buf[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0, tx_tail = 0;

volatile uint8_t rx_buf[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

// ubrr = (F_CPU / (16 * baud)) - 1.  For 9600 baud at 16 MHz: ubrr = 103.
void usartInit(uint16_t ubrr) {
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)(ubrr);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Non-blocking: returns false if the buffer has insufficient free space.
bool txEnqueue(const uint8_t *data, uint8_t len) {
	uint8_t freeSpace = (tx_tail - tx_head - 1) & TX_BUFFER_MASK;

	if (freeSpace < len) {
		return false;
	}

	uint8_t tempHead = tx_head;

	for (uint8_t i = 0; i < len; i++) {
		tx_buf[tempHead] = data[i];
		tempHead = (tempHead + 1) & TX_BUFFER_MASK;
	}

	tx_head = tempHead;
	UCSR0B |= (1 << UDRIE0);
	return true;
}
ISR(USART0_UDRE_vect) {
	UDR0 = tx_buf[tx_tail];
	tx_tail = (tx_tail + 1) & TX_BUFFER_MASK;

	if (tx_tail == tx_head) {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

bool rxDequeue(uint8_t *data, uint8_t len) {
	uint8_t available = (rx_head - rx_tail) & RX_BUFFER_MASK;

	if (available < len) {
		return false;
	}

	uint8_t tempTail = rx_tail;

	for (uint8_t i = 0; i < len; i++) {
		data[i] = rx_buf[tempTail];
		tempTail = (tempTail + 1) & RX_BUFFER_MASK;
	}

	rx_tail = tempTail;
	return true;
}

ISR(USART0_RX_vect) {
	uint8_t byte = UDR0;
	uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;

	if (next == rx_tail) {
		return;
	}

	rx_buf[rx_head] = byte;
	rx_head = next;
}
#endif

// =============================================================
// Framing: MAGIC_HI | MAGIC_LO | TPacket | XOR checksum
// =============================================================

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
	uint8_t cs = 0;
	for (uint8_t i = 0; i < len; i++)
		cs ^= data[i];
	return cs;
}

static void sendFrame(const TPacket *pkt) {
	uint8_t frame[FRAME_SIZE];
	frame[0] = MAGIC_HI;
	frame[1] = MAGIC_LO;
	memcpy(&frame[2], pkt, TPACKET_SIZE);
	frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);
#if USE_BAREMETAL_SERIAL
	while (!txEnqueue(frame, FRAME_SIZE))
		;
#else
	Serial.write(frame, FRAME_SIZE);
#endif
}

// Returns true only when *pkt holds a checksum-valid frame.
static bool receiveFrame(TPacket *pkt) {
#if USE_BAREMETAL_SERIAL
	while (((rx_head - rx_tail) & RX_BUFFER_MASK) >= FRAME_SIZE) {
		uint8_t hi = rx_buf[rx_tail];
		uint8_t lo = rx_buf[(rx_tail + 1) & RX_BUFFER_MASK];

		if (hi == MAGIC_HI && lo == MAGIC_LO) {
			uint8_t frame[FRAME_SIZE];
			for (uint8_t i = 0; i < FRAME_SIZE; i++)
				frame[i] = rx_buf[(rx_tail + i) & RX_BUFFER_MASK];

			uint8_t expected = computeChecksum(&frame[2], TPACKET_SIZE);
			if (frame[FRAME_SIZE - 1] == expected) {
				memcpy(pkt, &frame[2], TPACKET_SIZE);
				rx_tail = (rx_tail + FRAME_SIZE) & RX_BUFFER_MASK;
				return true;
			}
		}

		rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
	}
	return false;
#else
	static uint8_t state = 0;
	static uint8_t raw[TPACKET_SIZE];
	static uint8_t index = 0;

	while (Serial.available() > 0) {
		uint8_t byte = (uint8_t)Serial.read();

		switch (state) {
			case 0:
				if (byte == MAGIC_HI) {
					state = 1;
				}
				break;

			case 1:
				if (byte == MAGIC_LO) {
					state = 2;
					index = 0;
				} else if (byte != MAGIC_HI) {
					state = 0;
				}
				break;

			case 2:
				raw[index++] = byte;
				if (index >= TPACKET_SIZE) {
					state = 3;
				}
				break;

			case 3: {
					uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
					if (byte == expected) {
						memcpy(pkt, raw, TPACKET_SIZE);
						state = 0;
						return true;
					}
					state = (byte == MAGIC_HI) ? 1 : 0;
					break;
				}
		}
	}
	return false;
#endif
}
