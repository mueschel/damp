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

#define FREQMODE 1
#define AUDIOMODE 0
#define DACP OCR1A
#define DACN OCR1B
#define DACT TC1H

volatile uint8_t mode_select = AUDIOMODE; //1: frequency generator, 0: external sound input

volatile uint8_t amplitude = 1;  //amplitude
volatile uint16_t real_value_pos = 512;
volatile uint16_t real_value_neg = 512;

#define ADMUX_AUDIO         (1<<REFS0) | (1<<REFS1) | (0<<ADLAR) | 0   //input 0, reference internal 2.56V with capacitor
#define ADCSRA_AUDIO        (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (4<<ADPS0)
#define TCCR0B_AUDIO        0

#define ADMUX_FREQ          (0<<REFS0) | (0<<REFS1) | (0<<ADLAR) | 5 // PB3, AVcc, 8 bit, 5V, left aligned
#define ADCSRA_FREQ         (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (7<<ADPS0) // CPU/128 sampling rate (9.6kHz);
#define TCCR0B_FREQ         (3<<CS00)


__attribute__((naked)) int main(void)
{
    _delay_ms(500);

    //Port A is ADC only
    DIDR0  = 0xFF; //(1<<ADC6D) | (1<<ADC5D) | (1<<ADC0D); //disable digital input on ADC pins

	// ADC
	ADMUX  = ADMUX_AUDIO ;
	ADCSRA = ADCSRA_AUDIO; 
    ADCSRB = (1<<REFS2);

	// Timer0: Signal generation (CTC /256)
	OCR0A = 10; //interval to generate interrupt
    TCCR0A = (1<<0); //CTC mode  0: CTC0
    TCCR0B = TCCR0B_AUDIO;
    TIMSK |= (1<<OCIE0A); //interrupt on compare match / counter reset

    //Timer1: PWM
    TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (0<<PWM1A) | (0<<PWM1B) | (1<<FOC1A) | (1<<FOC1B);
    TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<PWM1A) | (1<<PWM1B) | (0<<FOC1A) | (0<<FOC1B);
    TCCR1B = (1<<7) | (1<<CS10); //  CLK/1  -> 64MHz/1/1024 = 62kHz
    TCCR1C |= 0b11110000;
    TCCR1D = 0;
    //TCCR1E = 0x0a;
    TC1H   = 0x3;  //set overflow for timer to 0x3ff
    OCR1C  = 0xFF;
    TC1H   = 0;
    //OCR1A  = 0x0F;
    //OCR1B  = 0x0F;
    //TCNT1  = 0;
    PLLCSR = (1<<PCKE) | (1<<PLLE);
    TIMSK |= (1<<TOIE1);

	// Configure ports
	DDRB  = 0b00011010;
	PORTB = 0b11100001;

	sei();
	while(1) {
		_delay_us(100);
        
        if(mode_select != ((~PINB) & 1) ) {
            if(mode_select == AUDIOMODE) {  //change to generator
                mode_select = FREQMODE;
                TCCR0B = TCCR0B_FREQ;
	            ADMUX  = ADMUX_FREQ;
	            ADCSRA = ADCSRA_FREQ;
                }
            else {                  //change to Audio
                mode_select = AUDIOMODE;
                TCCR0B = TCCR0B_AUDIO;
	            ADMUX  = ADMUX_AUDIO;
	            ADCSRA = ADCSRA_AUDIO;
                }
            }
	    }
    }

//mode 1: switch between ADC 5 (vol) and 6 (freq), set frequency and amplitude registers, two reads per input
//mode 0: copy ADC value to real_value for PWM
ISR(ADC_vect) {
    static uint16_t period = 160;
	static uint8_t channel = 5;
	static uint8_t valid = 0; //accept only second read from each input.
	if(mode_select == FREQMODE) { // read 
		if(valid == 0) {
			valid = 1;
		} else {		
            sei();
			valid = 0;
			if (channel==6) {
    			ADMUX = (ADMUX & 0xF0) | 5;
				period -= period/4;
				period += ADC;
				if (period < 240) period = 240;  //upper limit to prevent too high interrupt load
				OCR0A = period/16;
				channel = 5;
			} else {
    			ADMUX = (ADMUX & 0xF0) | 6;
				amplitude = ADC >> 2;
				channel = 6;
	    		}		
		    }	
	    }		
    else {
        uint16_t real_value;
        real_value     = ADC;
	    real_value_pos = real_value;
	    real_value_neg = 1023 - real_value;

        }
		
}	

//called at F_CPU/64*period, run only every second time.
//10<period<255 --> 3.8 - 97 Hz
ISR(TIMER0_COMPA_vect) {
	sei();
    static uint8_t time = 0;
	static int8_t direction = 1;  //ramp up or down
    static int8_t value = 0;
    static uint8_t cur_amplitude = 0;
    static int16_t tmp_value = 0;
    
    if(++time & 1)
        return;

	//real_value = ((int16_t)value * cur_amplitude) / 8;
    if (value == 0) {
        tmp_value = 0; //reset to correct 0 value
        cur_amplitude = amplitude;
        }
    else {
        if (direction == 1)
            tmp_value = tmp_value + cur_amplitude;
        else
            tmp_value = tmp_value - cur_amplitude;
        }           

	if(value <= -63 || value >= 63)
    	direction = -direction;
	value = value + direction;

    uint16_t real_value = 1024 + tmp_value /16;
    uint16_t val1 = real_value >> 1;
    uint16_t val2 = real_value & 1;
    if(val1 > 1023)
        val1 = 1023;
    if(val1 < 0)
        val1 = 0;
    real_value_pos = val1 + val2;
    real_value_neg = 1023 - val1;
}

	

ISR(TIMER1_OVF_vect) {
    uint16_t pos;
    uint16_t neg;

    pos = real_value_pos;
    neg = real_value_neg;

    DACT = (pos >> 8);
	DACP = pos;

    DACT = (neg >> 8);
	DACN = neg;

}

