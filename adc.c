#include <avr/io.h>

#include "adc.h"

void adc_init(void)
{
    // todo: set up proper values in the ADMUX and ADCSRA registers

    // step 1 - set/clear REF[1:0] bits to choose AVCC to give analog range of 0-5V
    // ADMUX register bit REFS1 to 0
    ADMUX &= ~(1 << REFS1);
    // ADMUX reg bit REFS0 to 1
    ADMUX |= (1 << REFS0);

    // step 2 - set prescalar to 128 by setting ADPS[2:0] bits in ADCSRA register
    ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));

    // step 3 - we are using 8-bit readings, se set ADLAR bit to 1 in ADMUX register
    // note: (1 = 8 bit results, 0 = 10-bit results)
    ADMUX |= (1 << ADLAR);

    // step 4 - set ADEN bit to 1 (enable bit which is on / off switch, we turn off to save power ig)
    ADCSRA |= (1 << ADEN);
}

// called to acquire a sample of the analog signal from the ADC
uint8_t adc_sample(uint8_t channel)
{
    // Set/clear the MUX[3:0] bits in ADMUX to select the input channel as specified by the argument (copy lower 4)
    uint8_t MASK = (channel & 0xF);
    ADMUX &= 0xF0;
    ADMUX |= MASK;

    // Set the ADSC bit in the ADCSRA register to a 1 - "start" bit
    ADCSRA |= (1 << ADSC);

    // when ADC is done it will AUTOMATICALLY clear the ADSC bit to 0, check if turned back to 0
    unsigned char result;
    while ((ADCSRA & (1 << ADSC)) != 0)
    {
        // just can let this cycle
    }
    result = ADCH;

    // Convert an analog input and return the 8-bit result
    return result;
}
