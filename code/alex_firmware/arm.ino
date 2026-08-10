// =============================================================
// Robot arm (Timer 5 servo driver)
// =============================================================

volatile int arm_curr_ticks[4];
volatile int arm_stage = 0;

static int constrainAngle(int idx, int angle) {
	switch (idx) {
		case BASE_IDX:
			return constrain(angle, BASE_MIN, BASE_MAX);
		case SHLD_IDX:
			return constrain(angle, SHLD_MIN, SHLD_MAX);
		case ELBW_IDX:
			return constrain(angle, ELBW_MIN, ELBW_MAX);
		case GRIP_IDX:
			return constrain(angle, GRIP_MIN, GRIP_MAX);
		default:
			return constrain(angle, 0, 180);
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
		arm_curr_ticks[i] = angleToPulse(constrainAngle(
					i, (int[]){BASE_HOME, SHLD_HOME, ELBW_HOME, GRIP_HOME}[i]));
		arm_target_ticks[i] = arm_curr_ticks[i];
	}

	cli();
	TCCR5A = 0;
	TCCR5B = 0;
	TCNT5 = 0;
	OCR5A = 39999; // 20ms cycle at prescaler 8 / 16MHz
	OCR5B = 0;
	TCCR5B |= (1 << WGM52);
	TCCR5B |= (1 << CS51);
	TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);
	sei();
}

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

ISR(TIMER5_COMPA_vect) { lerpTicks(); }

// Staggered servo pulses: stage 0-7 alternate rise/fall for each of the four
// servos at their CHECKPOINT offsets within the 20ms COMPA period.
ISR(TIMER5_COMPB_vect) {
	switch (arm_stage) {
		case 0: PORTC |=  (1 << BASE_PIN); OCR5B += arm_curr_ticks[BASE_IDX]; break;
		case 1: PORTC &= ~(1 << BASE_PIN); OCR5B  = SHLD_CHECKPOINT;          break;
		case 2: PORTC |=  (1 << SHLD_PIN); OCR5B += arm_curr_ticks[SHLD_IDX]; break;
		case 3: PORTC &= ~(1 << SHLD_PIN); OCR5B  = ELBW_CHECKPOINT;          break;
		case 4: PORTC |=  (1 << ELBW_PIN); OCR5B += arm_curr_ticks[ELBW_IDX]; break;
		case 5: PORTC &= ~(1 << ELBW_PIN); OCR5B  = GRIP_CHECKPOINT;          break;
		case 6: PORTC |=  (1 << GRIP_PIN); OCR5B += arm_curr_ticks[GRIP_IDX]; break;
		case 7: PORTC &= ~(1 << GRIP_PIN); OCR5B  = BASE_CHECKPOINT; arm_stage = -1; break;
	}
	arm_stage++;
}
