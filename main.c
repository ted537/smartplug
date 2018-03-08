#include <avr/io.h>
#include <util/delay.h>

// these values are all wrong right now

#define TEMP_PIN_A 1 // temp value
#define SWITCH_PIN_D 2 // digital switch
#define KNOB_PIN_A 3 // analogue knob

typedef struct {
	// basically a bool
	int switchon;
	// plop data in here
} pininputs;

void adjustwithtemp();
void adjustwithouttemp();
pininputs readpins();

static inline void initADC0(void) {
	ADMUX  |= (1 << REFS0); // reference voltage on AVCC
	ADCRSA |= (1 << ADPS2); // ADC clock prescaler (16)
	ADCSRA |= (1 << ADEN);  // enable ADC
}

int main(void) {
	initADC0();
	// initialize pullup resistor on the switch pin
	PORTD |= (1<<SWITCH_PIN_D);
	while (1) {
		// start ADC up
		ADCSRA |= (1<<ADSC);
		// gotta wait until the ADC does it thing
		loop_until_bit_is_clear(ADCSRA,ADSC);
		// read ADC
		adcValue = ADC;

		pininputs inputs = readpins();
		if (inputs.switchon) {
			adjustwithtemp();	
		}
		else {
			adjustwithouttemp();
		}
		_delay_ms(50); // wait a bit
	}
}

pininputs readpins(void) {
	pininputs rtrnme;
	rtrnme.switchon = bit_is_clear(PIND,SWITCH_PIN_D);
	uint8_t tempvalue;
	return rtrnme;
}

void adjustwithtemp(void) {

}

void adjustwithouttemp(void) {

}
