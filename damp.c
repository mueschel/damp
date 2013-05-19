#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdint.h>

//PB0  input select
//PB1  OC1A Output1
//PB2
//PB3  OC1B Output2
//PB4  LED
//PB6  input onoff
//PB7  reset

//PA0 ADC0  Audio In
//PA6 ADC5  volume
//PA7 ADC6  frequency

//CKSEL fuses should be 0b0001 - PLL as source


#define DACP OCR1A
#define DACN OCR1B
#define DACT TC1H

uint8_t mode_select = 0; //0: frequency generator, 1: external sound input

volatile int8_t value = 0;  //DAC uses upper 10 bits
volatile uint16_t period = 48;    //Compare register for frequency setting. 128 overflows to one period
volatile uint8_t amplitude = 1;  //amplitude
volatile int16_t real_value = 0;


__attribute__((naked)) int main(void)
{
	// Configure ports
	DDRB  = 0b00011010;
	PORTB = 0b11100101;
    //Port A is ADC only

	// ADC
	ADMUX  = (0<<REFS0) | (0<<REFS1) | (1<<ADLAR) | 5 ; // PB3, AVcc, 8 bit, 2.56V, left aligned
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (7<<ADPS0); // CPU/128 sampling rate (9.6kHz);
    ADCSRB = (1<<REFS2);
    DIDR0  = (1<<ADC6D) | (1<<ADC5D) | (1<<ADC1D); //disable digital input on ADC pins

	// Power reduction
	PRR   = 0b00000000;
	MCUCR = 0b00000000;


	// Timer0: Signal generation (CTC /256)
	OCR0A = period; //interval to generate interrupt
    TCCR0A = (1<<0); //CTC mode  0: CTC0
    TCCR0B = 0b00000100; // /256
    TIMSK |= (1<<OCIE0A); //interrupt on compare match / counter reset

    //Timer1: PWM
    TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<PWM1A) | (1<<PWM1B);
    TCCR1B = (1<<CS10); //  CLK/1  -> 64MHz/1/1024 = 62kHz
    TCCR1C = 0;
    TCCR1D = 0;
    TCCR1E = 0;
    PLLCSR = (1<<PCKE) | (1<<PLLE);
    TIMSK |= (1<<TOIE1);

	sei();
	while(1) {
		_delay_us(.10);
	}
}

//switch between ADC 5 (vol) and 6 (freq), set frequency and amplitude registers, two reads per input
ISR(ADC_vect) {
	sei();
	static uint8_t channel = 5;
	static uint8_t valid = 0; //accept only second read from each input.
	if(mode_select == 0) { // read 
		if(valid == 0) {
			valid = 1;
		} else {		
			valid = 0;
			if (channel==5) {
    			ADMUX = (ADMUX & 0xF0) | 6;
				period -= period/4;
				period += ADCH;
				if (period < 40) period = 40;  //upper limit to prevent too high interrupt load
				OCR0A = period/4;
				channel = 6;
			} else {
    			ADMUX = (ADMUX & 0xF0) | 5;
				amplitude = ADCH;
				channel = 5;
			}		
		}	
	}		
		
}	

//called at F_CPU/256*period 
//40<period<255 --> 3.8 - 97 Hz
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
	int16_t val1	= real_value;
    uint16_t pos;
    uint16_t neg;

	if(val1 > 511)
	    val1 = 511;

	if(val1 < -511)
	    val1 = -511;

	pos = 512 + val1;
	neg = 512 - val1;


    DACT = (pos >> 8) & 0x3;
	DACP = pos;

    DACN = (neg >> 8) & 0x3;
	DACN = neg;
}

