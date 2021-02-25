#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#define PIN7 (1 << 7)
#define PIN6 (1 << 6)
#define PIN5 (1 << 5)
#define PIN4 (1 << 4)
#define PIN3 (1 << 3)
#define PIN2 (1 << 2)
#define PIN1 (1 << 1)
#define PIN0 (1 << 0)
#define DUTY 200

void setup() {
  DDRD |= (PIN5);
  DDRB |= (PIN3);
  TCNT0 = 0;
  TCNT2 = 0;
  TCCR0A = 0b00100001;
  TCCR2A = 0b10000001;
  TIMSK0 |= 0b110; 
  TIMSK2 |= 0b110;
  //OCR0A = DUTY;
  OCR0B = DUTY;
  OCR2A = DUTY;
  //OCR2B = DUTY;
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000100;
  sei();
}

ISR(TIMER0_COMPA_vect)
{
}

ISR(TIMER0_COMPB_vect)
{
}

void loop() {
}
