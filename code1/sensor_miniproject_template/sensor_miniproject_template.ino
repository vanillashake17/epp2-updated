/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Motor durations for timed movement
#define MOVE_DURATION_MS 2000
#define TURN_DURATION_MS 2000

// Motor functions (forward, backward, ccw, cw, stop) are provided
// by robotlib.ino which is compiled together with this sketch.

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
  TPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.packetType = PACKET_TYPE_RESPONSE;
  pkt.command = resp;
  pkt.params[0] = param;
  sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
  sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool stateChanged = false;
volatile bool _pressedWhileStopped = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

// =============================================================
// Color sensor (TCS3200)
// =============================================================
#define THRESHOLD 20 // 50 x 100us = 5 ms debounce

volatile unsigned long _timerTicks = 0;
volatile unsigned long _lastTime = 0;

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */

// ---------------- COLOR SENSOR PINS ----------------

#define S0 (1 << PA0)         // Arduino D22
#define S1 (1 << PA1)         // Arduino D23
#define S2 (1 << PA2)         // Arduino D24
#define S3 (1 << PA3)         // Arduino D25
#define SENSOR_OUT (1 << PA4) // Arduino D26

// ---------------- SENSOR MEASUREMENT ----------------

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {

  if (s2)
    PORTA |= S2;
  else
    PORTA &= ~S2;
  if (s3)
    PORTA |= S3;
  else
    PORTA &= ~S3;

  uint32_t count = 0;
  unsigned long start = _timerTicks;

  while ((_timerTicks - start) < 1000) {
    if (PINA & SENSOR_OUT) {
      count++;
      while (PINA & SENSOR_OUT)
        ;
    }
  }

  return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
  *r = measureChannel(0, 0) * 10;
  *g = measureChannel(1, 1) * 10;
  *b = measureChannel(0, 1) * 10;
}

static void setupTimer() {
  cli();
  TCCR2A = (1 << WGM21); // CTC mode
  TCCR2B = 0;            // no clock yet
  OCR2A = 199;
  TIMSK2 = (1 << OCIE2A);
  TCNT2 = 0;
  sei();
}

static void startTimer() {
  TCCR2B |= (1 << CS21); // prescaler 8 → START TIMER
}

ISR(TIMER2_COMPA_vect) { _timerTicks++; }

ISR(INT1_vect) {
  unsigned long now = _timerTicks;

  if ((now - _lastTime) > THRESHOLD) {
    bool pressed = !(PIND & (1 << PD1)); // LOGIC low button

    if (buttonState == STATE_RUNNING && pressed) {
      buttonState = STATE_STOPPED;
      stateChanged = true;
      _pressedWhileStopped = false;
    } else if (buttonState == STATE_STOPPED && pressed) {
      // STOPPED + press -> stay STOPPED, but mark that a press occurred
      _pressedWhileStopped = true;
    } else if (buttonState == STATE_STOPPED && !pressed &&
               _pressedWhileStopped) {
      buttonState = STATE_RUNNING;
      stateChanged = true;
      _pressedWhileStopped = false;
    }

    _lastTime = now;
  }
}

// =============================================================
// Robot arm (Timer 5 servo driver)
// =============================================================

// PORTC bit positions (physical wiring)
#define BASE_PIN 0  // PC0 = Arduino 37
#define SHLD_PIN 1  // PC1 = Arduino 36
#define ELBW_PIN 4  // PC4 = Arduino 33
#define GRIP_PIN 2  // PC2 = Arduino 35

// array indices (must match ISR stage order)
#define BASE_IDX 0
#define SHLD_IDX 1
#define ELBW_IDX 2
#define GRIP_IDX 3

// servo pulse range in microseconds
#define MIN_PULSE 600
#define MAX_PULSE 2400

// empirically tested servo limits (degrees)
#define BASE_MIN 130
#define BASE_MAX 180
#define SHLD_MIN 110
#define SHLD_MAX 180
#define ELBW_MIN 0
#define ELBW_MAX 50
#define GRIP_MIN 5
#define GRIP_MAX 33

// default home pose (degrees)
#define BASE_HOME 180
#define SHLD_HOME 110
#define ELBW_HOME 50
#define GRIP_HOME 25

// staggered checkpoints within the 20ms period (timer ticks)
#define BASE_CHECKPOINT 0
#define SHLD_CHECKPOINT 10000
#define ELBW_CHECKPOINT 20000
#define GRIP_CHECKPOINT 30000

// lerp speed: ticks per 20ms period per servo
#define TICKS_PER_PERIOD 50  // tune this — higher = faster slew

volatile int arm_curr_ticks[4];
volatile int arm_target_ticks[4];
volatile int arm_stage = 0;

static int constrainAngle(int idx, int angle) {
  switch (idx) {
  case BASE_IDX: return constrain(angle, BASE_MIN, BASE_MAX);
  case SHLD_IDX: return constrain(angle, SHLD_MIN, SHLD_MAX);
  case ELBW_IDX: return constrain(angle, ELBW_MIN, ELBW_MAX);
  case GRIP_IDX: return constrain(angle, GRIP_MIN, GRIP_MAX);
  default:        return constrain(angle, 0, 180);
  }
}

static int angleToPulse(int angle) {
  int us = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
  return us * 2; // 0.5us per tick at prescaler 8 / 16MHz
}

// set target for one servo (degrees) — converts to ticks once here
static void setArmTarget(int idx, int angle) {
  int clamped = constrainAngle(idx, angle);
  arm_target_ticks[idx] = angleToPulse(clamped);
}

static void homeArm() {
  setArmTarget(BASE_IDX, BASE_HOME);
  setArmTarget(SHLD_IDX, SHLD_HOME);
  setArmTarget(ELBW_IDX, ELBW_HOME);
  setArmTarget(GRIP_IDX, GRIP_HOME);
}

static void setupArmTimer() {
  DDRC |= 0x17; // PC0-PC2, PC4 as outputs
  PORTC &= ~0x17;

  // initialise current ticks to home position
  for (int i = 0; i < 4; i++) {
    arm_curr_ticks[i]   = angleToPulse(constrainAngle(i, 
                            (int[]){BASE_HOME, SHLD_HOME, ELBW_HOME, GRIP_HOME}[i]));
    arm_target_ticks[i] = arm_curr_ticks[i];
  }

  cli();
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  OCR5A  = 39999; // 20ms cycle
  OCR5B  = 0;
  TCCR5B |= (1 << WGM52); // CTC mode
  TCCR5B |= (1 << CS51);  // prescaler 8
  TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);
  sei();
}

// called every 20ms from COMPA — lerp each servo one step toward target
static void lerpTicks() {
  for (int i = 0; i < 4; i++) {
    if (arm_curr_ticks[i] < arm_target_ticks[i]) {
      if (arm_curr_ticks[i] + TICKS_PER_PERIOD > arm_target_ticks[i])
        arm_curr_ticks[i] = arm_target_ticks[i];
      else
        arm_curr_ticks[i] += TICKS_PER_PERIOD;
    } else {
      if (arm_curr_ticks[i] - TICKS_PER_PERIOD < arm_target_ticks[i])
        arm_curr_ticks[i] = arm_target_ticks[i];
      else
        arm_curr_ticks[i] -= TICKS_PER_PERIOD;
    }
  }
}

// 20ms period — lerp step (stage reset is handled inside COMPB case 7)
ISR(TIMER5_COMPA_vect) {
  lerpTicks();
}

// staggered servo pulses
ISR(TIMER5_COMPB_vect) {
  switch (arm_stage) {
  case 0:
    PORTC |= (1 << BASE_PIN);
    OCR5B += arm_curr_ticks[BASE_IDX];
    break;
  case 1:
    PORTC &= ~(1 << BASE_PIN);
    OCR5B = SHLD_CHECKPOINT;
    break;
  case 2:
    PORTC |= (1 << SHLD_PIN);
    OCR5B += arm_curr_ticks[SHLD_IDX];
    break;
  case 3:
    PORTC &= ~(1 << SHLD_PIN);
    OCR5B = ELBW_CHECKPOINT;
    break;
  case 4:
    PORTC |= (1 << ELBW_PIN);
    OCR5B += arm_curr_ticks[ELBW_IDX];
    break;
  case 5:
    PORTC &= ~(1 << ELBW_PIN);
    OCR5B = GRIP_CHECKPOINT;
    break;
  case 6:
    PORTC |= (1 << GRIP_PIN);
    OCR5B += arm_curr_ticks[GRIP_IDX];
    break;
  case 7:
    PORTC &= ~(1 << GRIP_PIN);
    OCR5B = BASE_CHECKPOINT;
    arm_stage = -1;
    break;
  }
  arm_stage++;
}

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
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

    // ---------------- COLOR SENSOR COMMAND ----------------

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
    case 'B': setArmTarget(BASE_IDX, val); break;
    case 'S': setArmTarget(SHLD_IDX, val); break;
    case 'E': setArmTarget(ELBW_IDX, val); break;
    case 'G': setArmTarget(GRIP_IDX, val); break;
    case 'H': homeArm(); break;
    }

    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command = RESP_ARM;
    for (int i = 0; i < 4; i++)
      pkt.params[i] = (uint32_t)arm_curr_ticks[i];
    sendFrame(&pkt);
    break;
  }

  } // end switch
} // end handleCommand

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {

#if USE_BAREMETAL_SERIAL
  usartInit(103);
#else
  Serial.begin(9600);
#endif

  // ----------- COLOR SENSOR PIN SETUP -----------
  DDRA |= S0 | S1 | S2 | S3; // outputs
  DDRA &= ~SENSOR_OUT;       // input
  DDRD &= ~(1 << PD1);       // input (button pin)

  PORTA |= S0; // S0=HIGH, S1=LOW -> 20% frequency scaling
  PORTA &= ~S1;
  setupTimer();
  startTimer();
  EICRA |= (1 << ISC10); // trigger on any logical change
  EIMSK |= (1 << INT1);  // enable external interrupt 1

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
  // updateArmMovement() removed — lerp now runs from COMPA ISR
}
