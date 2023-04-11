#include "Debouncer.h"

void Debouncer::loop()
{
	state = digitalRead(pin);

	if (state != lastFlickerableState) {
		lastChangeTime = millis();
		lastFlickerableState = state;
	} 
	
	if ((millis() - lastChangeTime) > debounceTime) {
		if (trigger != 2) {
	  	if (lastState == !trigger && state == trigger) {
				callback();
			}
		} else {
			if (lastState != state) {
				callback();
			}
		}
		lastState = state;
	}
}

void Debouncer::setPinMode(uint8_t pin, uint8_t mode)
{
	this->pin = pin;
	pinMode(pin, mode);
	
}

void Debouncer::setTrigger(uint8_t trigger) {
	this->trigger = trigger;
	if (trigger == 2) {
		this->lastState = 2;
		this->lastFlickerableState = 2;
	} else {
	this->lastState = !trigger;
	this->lastFlickerableState = !trigger;
	}
}

void Debouncer::setDebounceTime(uint16_t debounceTime) {
	this->debounceTime = debounceTime;
}

void Debouncer::setCallback(void (*callback)())
{
	this->callback = callback;
}

uint8_t Debouncer::getState() {
	return state;
}
