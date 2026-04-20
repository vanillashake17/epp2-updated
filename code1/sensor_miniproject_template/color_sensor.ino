// =============================================================
// Color sensor (TCS3200)
// =============================================================

volatile unsigned long _timerTicks = 0;

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
			while (PINA & SENSOR_OUT);
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
	TCCR2A = (1 << WGM21);
	TCCR2B = 0;
	OCR2A = 199;
	TIMSK2 = (1 << OCIE2A);
	TCNT2 = 0;
	sei();
}

static void startTimer() {
	TCCR2B |= (1 << CS21);
}

ISR(TIMER2_COMPA_vect) { _timerTicks++; }
