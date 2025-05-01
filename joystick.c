#include <xc.h>

void InitializeADC1() {
    // Clear power-reduction bit for ADC
    PRR0 &= ~(1 << PRADC);
    
    // Select Vref = AVcc;
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    
    // Set the ADC clock div by 128
    // 16M / 128 = 125 kHz
    ADCSRA |= (1 << ADPS0);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS2);

    // Free running mode ADTS[2:0] = 000
    ADCSRB &= ~(1<<ADTS0);
    ADCSRB &= ~(1<<ADTS1);
    ADCSRB &= ~(1<<ADTS2);
}

// need to read both X and y values so ADC needs to be set 2 times per polling
int readADC1(){
//    if (channel == 0){
         // Select Channel ADC0 (pin C0)
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);

    ADCSRA |= (1 << ADATE); // Autotriggering of ADC

    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC0D);

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    return ADC;
}