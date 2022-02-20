#include <Arduino.h>
#include <avr/io.h>
#include <stdbool.h>

uint16_t adcVal;

void ADC0_init(void);
uint16_t ADC0_read(void);
void ADC0_start(void);
bool ADC0_conversionDone(void);
void ADC0_init(void){
  
  //disable digital input buffer
  PORTD.PIN4CTRL &= ~PORT_ISC_gm;
  PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;

  //disable pull-up resistor
  PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;

  //           divide clock by 4 | set reference voltage to VDD
  ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_VDDREF_gc;

  //            Enable ADC    | 10 bit mode
  ADC0.CTRLA =  ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;

  //            Select ADC channel - using Arduino Nano Every pin A6 which is PD4 which is channel AIN4
  ADC0.MUXPOS = ADC_MUXPOS_AIN4_gc;

  //            Enable freerun mode
  ADC0.CTRLA |= ADC_FREERUN_bm;
}

 uint16_t ADC0_read(void){
  //            clear interrupt flag by writing 1
    ADC0.INTFLAGS = ADC_RESRDY_bm;

  //            return value stored in result register  
    return ADC0.RES;
  }

void ADC0_start(void){
  //            start conversion
  ADC0.COMMAND = ADC_STCONV_bm;
}

bool ADC0_conversionDone(void){
  //         returns 1 if result ready interrupt flag is set    
  return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}


void setup() {

  Serial.begin(9600);
  ADC0_init();
  ADC0_start();
}

void loop() {
  
  if(ADC0_conversionDone()){
    adcVal = ADC0_read();
  }
  Serial.println(adcVal);
}
