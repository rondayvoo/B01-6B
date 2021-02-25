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
  DDRD |= (PIN5 | PIN6);
  DDRB |= (PIN2 | PIN3);
  TCNT0 = 0;
  TCNT1 = 0;
  TCNT2 = 0;
  TCCR0A = 0b10100001; 
  TCCR1A = 0b00100011; 
  TCCR2A = 0b10000001;
  TIMSK0 |= 0b011; 
  TIMSK1 |= 0b100;
  TIMSK2 |= 0b100;

  //Uncomment the next two lines to move forward 
  //OCR0A = DUTY;
  //OCR1B = DUTY << 8;

  //Uncomment the next two lines to move in reverse
  //OCR0B = DUTY;
  //OCR2A = DUTY;
  
  TCCR0B = 0b00000011;
  TCCR1B = 0b00010011;
  TCCR2B = 0b00000100;
  sei();
}

ISR(TIMER0_COMPA_vect)
{
}

ISR(TIMER0_COMPB_vect)
{
}

ISR(TIMER1_COMPB_vect)
{
}

ISR(TIMER2_COMPA_vect)
{
}

void loop() {
}
