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

uint8_t mode_select = 0; //1: frequency generator, 0: external sound input

volatile uint16_t period = 48;    //Compare register for frequency setting. 128 overflows to one period
volatile uint8_t amplitude = 1;  //amplitude
volatile int16_t real_value = 0;

#define ADMUX_AUDIO         (1<<REFS0) | (1<<REFS1) | (0<<ADLAR) | 0   //input 0, reference internal 2.56V with capacitor
#define ADCSRA_AUDIO        (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (4<<ADPS0)
#define TCCR0B_AUDIO        0

#define ADMUX_FREQ          (0<<REFS0) | (0<<REFS1) | (1<<ADLAR) | 5 // PB3, AVcc, 8 bit, 2.56V, left aligned
#define ADCSRA_FREQ         (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (7<<ADPS0) // CPU/128 sampling rate (9.6kHz);
#define TCCR0B_FREQ         (4<<CS00)


__attribute__((naked)) int main(void)
{
	// Configure ports
	DDRB  = 0b00010000;
	PORTB = 0b11101111;
    //Port A is ADC only
    //DIDR0  = 0xFF; //(1<<ADC6D) | (1<<ADC5D) | (1<<ADC0D); //disable digital input on ADC pins

	// ADC
	ADMUX  = ADMUX_AUDIO ;
	ADCSRA = ADCSRA_AUDIO; 
    ADCSRB = (1<<REFS2);

	// Timer0: Signal generation (CTC /256)
	OCR0A = period; //interval to generate interrupt
    TCCR0A = (1<<0); //CTC mode  0: CTC0
    TCCR0B = TCCR0B_AUDIO;
    TIMSK |= (1<<OCIE0A); //interrupt on compare match / counter reset

    //Timer1: PWM
    TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<PWM1A) | (1<<PWM1B);
    TCCR1B = (1<<CS10); //  CLK/1  -> 64MHz/1/1024 = 62kHz
    TCCR1C |= 0b11000000;
    TCCR1D = 0;
    //TCCR1E = 0x0a;
    TC1H   = 0x3;  //set overflow for timer to 0x3ff
    OCR1C  = 0xFF;
    TC1H   = 0;
    OCR1A  = 0x0F;
    OCR1B  = 0x0F;
    TCNT1  = 0;
    PLLCSR = (1<<PCKE) | (1<<PLLE);
    TIMSK |= (1<<TOIE1);

	sei();
	while(1) {
		_delay_us(100);/*
        
        if(mode_select != ((~PINB) & 1) ) {
            if(mode_select == 0) {  //change to generator
                mode_select = 1;
                TCCR0B = TCCR0B_FREQ;
	            ADMUX  = ADMUX_FREQ;
	            ADCSRA = ADCSRA_FREQ;
                }
            else {                  //change to Audio
                mode_select = 0;
                TCCR0B = TCCR0B_AUDIO;
	            ADMUX  = ADMUX_AUDIO;
	            ADCSRA = ADCSRA_AUDIO;
                }
            }*/
	    }
    }

//mode 1: switch between ADC 5 (vol) and 6 (freq), set frequency and amplitude registers, two reads per input
//mode 0: copy ADC value to real_value for PWM
ISR(ADC_vect) {
	sei();
	static uint8_t channel = 5;
	static uint8_t valid = 0; //accept only second read from each input.
	if(mode_select == 1) { // read 
		if(valid == 0) {
			valid = 1;
		} else {		
			valid = 0;
			if (channel==6) {
    			ADMUX = (ADMUX & 0xF0) | 5;
				period -= period/4;
				period += ADCH;
				if (period < 40) period = 40;  //upper limit to prevent too high interrupt load
				OCR0A = period/4;
				channel = 5;
			} else {
    			ADMUX = (ADMUX & 0xF0) | 6;
				amplitude = ADCH;
				channel = 6;
	    		}		
		    }	
	    }		
    else {
        real_value = ADC;
        }
		
}	

//called at F_CPU/256*period 
//40<period<255 --> 3.8 - 97 Hz
ISR(TIMER0_COMPA_vect) {
	sei();
	static int8_t direction = 1;  //ramp up or down
    static int8_t value = 0;
    static uint8_t cur_amplitude = 0;

    // Amplitude changes at 0 only
    if (value == 0) 
        cur_amplitude = amplitude;

	value = value + direction;

#if 0 //standard arithmetic
	real_value = ((int16_t)value * cur_amplitude) / 4;


#else //optimized without multiplication.
    if (value == 0) {
        real_value = 0; //reset to correct 0 value
        }
    else if (direction == 1)
        real_value = real_value + cur_amplitude;
    else
        real_value = real_value - cur_amplitude;
    real_value >>= 2;
#endif

	if(value <= -31 || value >= 31)
    	direction = -direction;

}

	



ISR(TIMER1_OVF_vect) {
	int16_t val1	= real_value ;//>> 1;
    //uint8_t val2    = real_value & 1;
    uint16_t pos;
    uint16_t neg;
        PORTB ^= (1<<PB4);
	if(val1 > 511)
	    val1 = 511;
	else if(val1 < -511)
	    val1 = -511;

	pos = 512 + val1;//+ val2;
	neg = 512 - val1;

/*
    DACT = (pos >> 8);
	DACP = pos;

    DACT = (neg >> 8);
	DACN = neg;
*/
}

