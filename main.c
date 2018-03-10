// really useful link http://maxembedded.com/2011/06/the-adc-of-the-avr/
// microcontroller info http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf

// model of microcontroller
#define __AVR_ATmega328P__
// cpu rate (hz) (8MHz)
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

// these values are all wrong right now
// A suffix is analogue
// D suffix is digital
#define TEMP_PIN_1_A SDA
#define TEMP_PIN_2_A SCL
// not actually too sure about these ones
// if it doesn't work, try PD6 and/or PD7
#define SWITCH_PIN_1_D PB0
#define SWITCH_PIN_2_D PB1
#define DIAL_PIN_1_A PB2
#define DIAL_PIN_2_A PB3

typedef struct {
	// basically a bool
	int switchon_1,switchon_2;
	// plop data in here
} pininputs;

void adjustwithtemp_1(void);
void adjustwithtemp_2(void);
void adjustwithouttemp_1(void);
void adjustwithouttemp_2(void);
pininputs readpins(void);

// turns ADC on with correct settings by configuring bitmask
// info here http://www.robotplatform.com/knowledge/ADC/adc_tutorial_2.html
static inline void initADC0(void) {
	// ADMUX stands for ADC multiplexer selection register
	ADMUX  |= (1 << REFS0); // reference voltage on AVCC
	ADCSRA |= (1 << ADPS2); // ADC clock prescaler (16)
	ADCSRA |= (1 << ADEN);  // enable ADC
}

static inline void initSwitches(void) {

}

static inline void setupPins() {
	// setup pins as input or output
	DDRB = 0b00000000;
	DDRC = 0b00000000;
	DDRD = 0b11000000;
}

// need to add second one
int main(void) {
	setupPins();
	initADC0();
	while (1) {
		pininputs inputs = readpins();
		if (inputs.switchon_1) {
			adjustwithtemp_1();	
		}
		else {
			adjustwithouttemp_1();
		}
		if (inputs.switchon_2) {
			adjustwithtemp_1();
		}
		else {
			adjustwithouttemp_2();
		}
		_delay_ms(50); // wait a bit
	}
}

pininputs readpins(void) {
	// is there only one ADC pin?
	// how do we read input of both temp and dial?
	// gotta read successively i guess
	// start ADC up
	ADCSRA |= (1<<ADSC);
	// gotta wait until the ADC does it thing
	uint8_t adcValue = ADC;

	pininputs rtrnme;
	rtrnme.switchon_1 = bit_is_set(PIND,SWITCH_PIN_1_D);
	rtrnme.switchon_2 = bit_is_set(PIND,SWITCH_PIN_2_D);
	uint8_t tempvalue;
	return rtrnme;
}

void adjustwithtemp_1(void) {

}

void adjustwithtemp_2(void) {

}

void adjustwithouttemp_1(void) {

}

void adjustwithouttemp_2(void) {

}
