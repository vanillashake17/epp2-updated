#include "packets.h"
#include "serial_driver.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// =============================================================
// Constants
// =============================================================

#define MOVE_DURATION_MS 2000  // motor forward/backward duration
#define TURN_DURATION_MS 2000  // motor turn duration
#define THRESHOLD 20    // debounce threshold (20 x 100us = 2 ms)

#define S0         (1 << PA0)  // Arduino D22
#define S1         (1 << PA1)  // Arduino D23
#define S2         (1 << PA2)  // Arduino D24
#define S3         (1 << PA3)  // Arduino D25
#define SENSOR_OUT (1 << PA4)  // Arduino D26

#define BASE_PIN 0  // PC0 = Arduino 37
#define SHLD_PIN 1  // PC1 = Arduino 36
#define ELBW_PIN 4  // PC4 = Arduino 33
#define GRIP_PIN 2  // PC2 = Arduino 35

#define BASE_IDX 0  // array index for base servo
#define SHLD_IDX 1  // array index for shoulder servo
#define ELBW_IDX 2  // array index for elbow servo
#define GRIP_IDX 3  // array index for gripper servo

#define MIN_PULSE 600   // servo min pulse width (us)
#define MAX_PULSE 2400  // servo max pulse width (us)

#define BASE_MIN 0  // base angle lower limit
#define BASE_MAX 180  // base angle upper limit
#define SHLD_MIN 110  // shoulder angle lower limit
#define SHLD_MAX 180  // shoulder angle upper limit
#define ELBW_MIN 65    // elbow angle lower limit
#define ELBW_MAX 180   // elbow angle upper limit
#define GRIP_MIN 5    // gripper angle lower limit
#define GRIP_MAX 40   // gripper angle upper limit

#define BASE_HOME 80  // base home angle
#define SHLD_HOME 110  // shoulder home angle
#define ELBW_HOME 60   // elbow home angle
#define GRIP_HOME 15   // gripper home angle

#define BASE_CHECKPOINT 0      // staggered checkpoint (timer ticks)
#define SHLD_CHECKPOINT 10000  // staggered checkpoint (timer ticks)
#define ELBW_CHECKPOINT 20000  // staggered checkpoint (timer ticks)
#define GRIP_CHECKPOINT 30000  // staggered checkpoint (timer ticks)

#define TICKS_PER_PERIOD 50  // lerp speed: ticks per 20ms period per servo

// =============================================================
// Shared state (defined here so it is visible to every .ino in
// this folder via Arduino's concatenated translation unit).
//   arm.ino         writes arm_target_ticks
//   estop.ino       writes buttonState and stateChanged
//   color_sensor.ino owns _timerTicks
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool stateChanged = false;
volatile int arm_target_ticks[4];

// =============================================================
// Packet helpers
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
	TPacket pkt;
	memset(&pkt, 0, sizeof(pkt));
	pkt.packetType = PACKET_TYPE_RESPONSE;
	pkt.command = resp;
	pkt.params[0] = param;
	sendFrame(&pkt);
}

static void sendStatus(TState state) {
	sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
	if (cmd->packetType != PACKET_TYPE_COMMAND)
		return;

	switch (cmd->command) {
		case COMMAND_ESTOP:
			cli();
			buttonState = STATE_STOPPED;
			stateChanged = false;
			sei();
			{
				TPacket pkt;
				memset(&pkt, 0, sizeof(pkt));
				pkt.packetType = PACKET_TYPE_RESPONSE;
				pkt.command = RESP_OK;
				strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
				pkt.data[sizeof(pkt.data) - 1] = '\0';
				sendFrame(&pkt);
			}
			sendStatus(STATE_STOPPED);
			break;

		case COMMAND_COLOR: {

					    uint32_t r, g, b;
					    readColorChannels(&r, &g, &b);

					    TPacket pkt;
					    memset(&pkt, 0, sizeof(pkt));
					    pkt.packetType = PACKET_TYPE_RESPONSE;
					    pkt.command = RESP_COLOR;
					    pkt.params[0] = r;
					    pkt.params[1] = g;
					    pkt.params[2] = b;

					    sendFrame(&pkt);

					    break;
				    }

		case COMMAND_MOVE: {
					   if (buttonState != STATE_RUNNING) {
						   sendStatus(STATE_STOPPED);
						   break;
					   }

					   uint8_t speed = (uint8_t)cmd->params[0];
					   uint32_t duration = cmd->params[1];
					   char dir = cmd->data[0];

					   switch (dir) {
						   case 'w':
							   forward(speed);
							   break;
						   case 's':
							   backward(speed);
							   break;
						   case 'a':
							   ccw(speed);
							   break;
						   case 'd':
							   cw(speed);
							   break;
						   case 'x':
						   default:
							   stop();
							   sendResponse(RESP_OK, 0);
							   break;
					   }

					   if (dir == 'w' || dir == 's' || dir == 'a' || dir == 'd') {
						   unsigned long start = millis();
						   bool interrupted = false;
						   while (millis() - start < duration) {
							   TPacket incoming;
							   if (receiveFrame(&incoming)) {
								   if (incoming.packetType == PACKET_TYPE_COMMAND &&
										   incoming.command == COMMAND_MOVE && incoming.data[0] == 'x') {
									   interrupted = true;
									   break;
								   }
								   if (incoming.packetType == PACKET_TYPE_COMMAND &&
										   incoming.command == COMMAND_ESTOP) {
									   interrupted = true;
									   break;
								   }
							   }
							   if (buttonState != STATE_RUNNING) {
								   interrupted = true;
								   break;
							   }
						   }
						   stop();
						   sendResponse(RESP_OK, interrupted ? 1 : 0);
					   }
					   break;
				   }

		case COMMAND_ARM: {
					  char c = cmd->data[0];
					  int val = (int)cmd->params[0];

					  switch (c) {
						  case 'B':
							  setArmTarget(BASE_IDX, val);
							  break;
						  case 'S':
							  setArmTarget(SHLD_IDX, val);
							  break;
						  case 'E':
							  setArmTarget(ELBW_IDX, val);
							  break;
						  case 'G':
							  setArmTarget(GRIP_IDX, val);
							  break;
						  case 'H':
							  homeArm();
							  break;
					  }

					  TPacket pkt;
					  memset(&pkt, 0, sizeof(pkt));
					  pkt.packetType = PACKET_TYPE_RESPONSE;
					  pkt.command = RESP_ARM;
					  // Report the commanded target so the Pi / second terminal
					  // print matches where the servos will settle, not the
					  // pre-lerp position.
					  for (int i = 0; i < 4; i++)
						  pkt.params[i] = (uint32_t)arm_target_ticks[i];
					  sendFrame(&pkt);
					  break;
				  }

	}
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {

#if USE_BAREMETAL_SERIAL
	usartInit(103);
#else
	Serial.begin(9600);
#endif

	DDRA |= S0 | S1 | S2 | S3;
	DDRA &= ~SENSOR_OUT;
	DDRD &= ~(1 << PD1);

	PORTA |= S0; // S0=HIGH, S1=LOW -> 20% frequency scaling (TCS3200)
	PORTA &= ~S1;
	setupTimer();
	startTimer();
	EICRA |= (1 << ISC10);
	EIMSK |= (1 << INT1);

	setupArmTimer();

	sei();
}

void loop() {
	if (stateChanged) {
		cli();
		TState state = buttonState;
		stateChanged = false;
		sei();
		sendStatus(state);
	}

	TPacket incoming;
	if (receiveFrame(&incoming)) {
		handleCommand(&incoming);
	}
}
