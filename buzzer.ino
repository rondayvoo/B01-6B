#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PORTBPIN1MASK 1 << 1

int countUp = 1;
void setup() {
  // put your setup code here, to run once:

  TCNT1 = 0;
  TCCR1A = 0b10100001;

  OCR1AL = 196;
  TCCR1B = 0b00000010;
 
  DDRB |=PORTBPIN1MASK;
}

void loop() {
 /*
  //version 1
  if(OCR1AL==255||OCR1AL==128) {
    countUp = 1 - countUp;
    
  }
  if(countUp) {
    OCR1AL++;
  
  }
  else {
    OCR1AL--;
   
  }
*/

  //version 2
  OCR1AL = 255;
  delay(400);
  OCR1AL = 196;
  delay(400);
 
}
