// =============================================================
// E-Stop state machine (INT1, debounced against Timer 2 ticks)
// =============================================================

static volatile unsigned long _lastTime = 0;
static volatile bool _pressedWhileStopped = false;

ISR(INT1_vect) {
	unsigned long now = _timerTicks;

	if ((now - _lastTime) > THRESHOLD) {
		bool pressed = !(PIND & (1 << PD1)); // active-low button

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
