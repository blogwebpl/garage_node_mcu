#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#define BOTH 2

#include <Arduino.h>

class Debouncer
{
public:
	void setPinMode(uint8_t pin, uint8_t mode);
	void setTrigger(uint8_t trigger);
	void setDebounceTime(uint16_t debounceTime);
	void setCallback(void (*callback)());
	void loop();
	uint8_t getState();

private:
	uint8_t pin;
	uint8_t trigger;
	uint8_t debounceTime;
	uint8_t state;
	uint8_t lastState;
	uint8_t lastFlickerableState;
	unsigned long lastChangeTime;
	void (*callback)();
};

#endif
