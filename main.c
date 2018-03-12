// really useful link http://maxembedded.com/2011/06/the-adc-of-the-avr/
// microcontroller info http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf

// assuming 10 bit resolution in ADC

// model of microcontroller
#define __AVR_ATmega328P__
// cpu rate (hz) (8MHz)
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>


// these values are all wrong right now
// A suffix is analogue
// D suffix is digital
// these pins are probably wrong
#define TEMP_PIN_1_A PC2
#define TEMP_PIN_2_A PC3

#define SWITCH_PIN_1_D PB0
#define SWITCH_PIN_2_D PB1
// these pins are probably wrong
#define DIAL_PIN_1_A PC0
#define DIAL_PIN_2_A PC1

#define PWM_PIN_1 PD0
#define PWM_PIN_2 PD1

#define SELECT_ADC0 0b00000000
#define SELECT_ADC1 0b00000001
#define SELECT_ADC2 0b00000010
#define SELECT_ADC3 0b00000011
#define SELECT_ADC4 0b00000100
#define SELECT_ADC5 0b00000101
#define SELECT_ADC6 0b00000110
#define SELECT_ADC7 0b00000111

#define ADC_PRECISION 1024

// defines period of PWM in clock cycles
// higher is more accurate but slower
// lower is less accurate but faster
#define PWM_PERIOD 255

// WARNING GLOBAL VARIABLES AHEAD

// indexes for PWM
uint8_t pwm1_i,pwm2_i;

typedef struct {
	// basically a bool
	int switchon_1,switchon_2;

	// max value 1024 (10 bits)
	uint16_t temp1,temp2,dial1,dial2;
	// plop data in here
} pininputs;

void pwmpin(uint8_t *index, uint8_t pulsewidth, uint8_t pin);
void adjustwithtemp_1(void);
void adjustwithtemp_2(void);
void adjustwithouttemp_1(void);
void adjustwithouttemp_2(pininputs input);
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
	// 2 output pins
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
			adjustwithouttemp_2(inputs);
		}
		_delay_ms(50); // wait a bit
	}
}

pininputs readpins(void) {
	pininputs rtrnme;
	// is there only one ADC pin?
	// how do we read input of both temp and dial?
	// gotta read successively i guess

	// just numbers mate
	// to select bitwise just OR by the x in ADCx
	// (up to 7)
	
	ADMUX|=SELECT_ADC0;
	// get going
	ADCSRA |= (1<<ADSC);
	// wait til done
	loop_until_bit_is_clear(ADCSRA,ADSC);

	rtrnme.dial1 = ADC;

	ADMUX|=SELECT_ADC1;
	ADCSRA |= (1<<ADSC);
	loop_until_bit_is_clear(ADCSRA,ADSC);

	rtrnme.dial2 = ADC;

	ADMUX|=SELECT_ADC2;
	ADCSRA |= (1<<ADSC);
	loop_until_bit_is_clear(ADCSRA,ADSC);

	rtrnme.temp1 = ADC;

	ADMUX|=SELECT_ADC3;
	ADCSRA |= (1<<ADSC);
	loop_until_bit_is_clear(ADCSRA,ADSC);

	rtrnme.temp2 = ADC;

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
	uint8_t pulsewidth = input.dial1*PWM_PERIOD/ADC_PRECISION;
	uint8_t pin = PWM_PIN_1;
	pwmpin(&pwm1_i,pulsewidth,pin);
}

void adjustwithouttemp_2(pininputs input) {
	uint8_t pulsewidth = input.dial2*PWM_PERIOD/ADC_PRECISION;
	uint8_t pin = PWM_PIN_2;
	pwmpin(&pwm2_i,pulsewidth,pin);
}

// pin should should only have 1 1 bit
void pwmpin(uint8_t *index,uint8_t pulsewidth, uint8_t pin) {
	++*index;
	if (*index<pulsewidth) {
		// set pin bit
		PORTD|=pin;
	}
	else {
		// unset pin bit
		PORTD&=~pin;	
	}
	if (*index>=PWM_PERIOD) {
		*index=0;
	}
}
