#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdint.h>



#define DACP OCR1A
#define DACN OCR1B


uint8_t mode_select = 0; //0: frequency generator, 1: external sound input

volatile int8_t value = 0;  //DAC uses upper 10 bits
volatile uint16_t period = 48;    //Compare register for frequency setting. 128 overflows to one period
volatile uint8_t amplitude = 1;  //amplitude
volatile int16_t real_value = 0;

uint8_t timer;
uint8_t time;
uint16_t pos;
uint16_t neg;


__attribute__((naked)) int main(void)
{
	// Configure ports
	DDRB  = 0b00010010; // OC0AB are L/R PWM output. Drive unused pins low (helps noise).
	PORTB = 0b11100000; // No activate pull-ups


	// ADC
	ADMUX  = (0<<REFS0) | (0<<REFS1) | (1<<REFS2) | (1<<ADLAR) | 3 ; // PB3, AVcc, 8 bit, 2.56V, left aligned
	ADCSRA = 0b11101111; // CPU/128 sampling rate; Sampling freq: 16 MHz / 16 / 13 = 67 kHz

	// Power reduction
	PRR   = 0b00000000;
	MCUCR = 0b00000000;


	// Timer0: Signal generation (CTC /256)
	OCR0A = period; //interval to generate interrupt
	TCCR0A = 0b00000010; //CTC mode
	TCCR0B = 0b00000100; // /256
	TIMSK |= (1<<OCIE0A); //interrupt on compare match / counter reset

    //Timer1: PWM
	TCCR1 = 0b01100011; // non-inv PWM, 64MHz /4 / 512 = 62 kHz
	GTCCR = 0b01100000;
	TIMSK |= (1<<TOIE1);
	PLLCSR = 0b110;	
	
	sei();
	while(1) {
		_delay_us(.10);
	}
}

//switch between PB3 and PB4, set frequency and amplitude registers, two reads per input
ISR(ADC_vect) {
	sei();
	static uint8_t channel = 3;
	static uint8_t valid = 0; //accept only second read from each input.
	if(mode_select == 0) { // read 
		if(valid == 0) {
			valid = 1;
		} else {		
			valid = 0;
			ADMUX = ADMUX ^ 2;
			if (channel==3) {
				period -= period/4;
				period += ADCH;
				if (period < 40) period = 40;  //upper limit to prevent too high interrupt load
				OCR0A = period/4;
				channel = 1;
			} else {
				amplitude = ADCH;
				channel = 3;
			}		
		}	
	}		
		
}	

//called at F_CPU/256*period 
//9<period<255 --> 3.8 - 97 Hz
ISR(TIMER0_COMPA_vect) {
	sei();
	static int8_t direction = 1;  //ramp up or down
	value = value + direction;
	if(value <= -31 || value >= 31)
		direction = -direction;
	//calculate real 10 Bit value
	real_value = ((int16_t)value * amplitude) / 4;
}

	



ISR(TIMER1_OVF_vect) {
    uint8_t posn, negn;
	if(1 & ++time) {
		int16_t val1	= real_value >> 1;
		uint8_t val2	= (uint8_t)(real_value & 1);

		pos = 256 + val1 + val2;
		neg = 256 - val1;

		if(val1 >= 254)
		pos = 510;

		if(val1 <= -255)
		neg = 510;

		if(pos > 255) {
			posn = 255;
			pos -= 255;
		} else {
			posn = pos;
			pos = 0;
		}

		if(neg > 255) {
			negn = 255;
			neg -= 255;
		} else {
			negn = neg;
			neg = 0;
		}
		DACP = posn;
		DACN = negn;

	} else {
		DACP = pos;
		DACN = neg;
	}
}

