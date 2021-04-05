// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//buzzer
#define PORTBPIN1MASK 1 << 1

//ultrasound
#define trigPin 18 // PB4, A4
#define echoPin 19 // PB5, A5

//colour sensor
#define S0 4
#define S1 7
#define S2 8
#define S3 13
#define sensorOut 12

int frequencyR = 0;
int frequencyG = 0;
int frequencyB = 0;

int TotalR = 0;
int TotalG = 0;
int TotalB = 0;

int count = 0;

int averageR=0;
int averageG=0;
int averageB=0;

//buzzer
int countUp = 1;
int played = 0;

// defines variables
long duration = 0; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
unsigned long StartTime;
unsigned long CurrentTime;
bool stop = 0;
bool flag = false;
int colour = 0;

void setup() {
//buzzer

  TCNT1 = 0;
  TCCR1A = 0b10100001;

  OCR1AL = 0;
  TCCR1B = 0b00000010;
 
  DDRB |=PORTBPIN1MASK;
  
  //Ultrasound
  // Sets the trigPin (PB4) as an Output
  DDRC |= 1 << 4;
  /// Sets the echoPin (PB5) as Input
  DDRC |= 0 << 5;
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  //////////////////////////
  //Colour sensor
  /*
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  */
  //Set S0,S1,S2,S3 to be outputs
  DDRD |= (1<<S0 | 1<<S1);
  DDRB |= (1<<0 | 1<<5);
  
  //pinMode(sensorOut, INPUT);

  //Set sensorout to be input
  DDRB |= (0<<4);
  

  
  //Setting frequency-scaling to 20% (H,L)
  //Setting frequency-scaling to 100% (H,H)
  /*
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  */
  //set S0 and S1 to high
  PORTD |= (1<<S0 | 1<<S1);
  
  Serial.begin(9600);
}

void loop() {

    

/////////////////////////////
  
  // Clears the trigPin (PB4)
  PORTC &= ~(1 << 4);
  delayMicroseconds(2);
  
  // Sets the trigPin to HIGH for 10 microseconds, then sets to LOW
  PORTC |= 1 << 4;
  delayMicroseconds(10);
  PORTC &= ~(1 << 4);
 
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  if (distance>18)
  {
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm"); 
    played = 0;
    OCR1AL = 0;
  }
  
  if (distance <= 18)
  {
    count++;
  /*
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  */
  //Set S2 and S3 to low
  PORTB |= (0<<0 | 0<<5);
  
  // Reading the output frequency
  frequencyR = pulseIn(sensorOut, LOW);
  delay(10);
  
  // Setting Green filtered photodiodes to be read
  /*
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  */
  //Set S2 and S3 to high
  PORTB |= (1<<0 | 1<<5);
  
  // Reading the output frequency
  frequencyG = pulseIn(sensorOut, LOW);
  delay(10);
  
  // Setting Blue filtered photodiodes to be read
  /*
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  */
  //set S2 to low and S3 to high
  PORTB |= (0<<0 | 1<<5); 

  
  // Reading the output frequency
  frequencyB = pulseIn(sensorOut, LOW);

  TotalR+=frequencyR;
  TotalB+=frequencyB;
  TotalG+=frequencyG;

  if(count%10==0) {
//    Serial.print(count);
    averageR=TotalR/10;
    averageG=TotalG/10;   
    averageB=TotalB/10;

    //since the detector of colour sensor shows the inverse readings
    //use map to un-inverse it.Pre-adjusted values ranges from 0 to 170
    averageR = map(averageR,0,170,255,0);
    averageG = map(averageG,0,170,255,0);
    averageB = map(averageB,0,170,255,0);
    
    Serial.print("R= ");
    Serial.print(averageR);
    Serial.print(" G= ");
    Serial.print(averageG);
    Serial.print(" B= ");
    Serial.print(averageB);
    Serial.println("  ");
    
    if(averageG >160) {
      colour = 1;
       Serial.println("GREEN");
       if(!played)
                {
                OCR1AL = 200;
                played =1;
                }
         delay(1000);
         OCR1AL = 0;
    }

    else if(averageR>140 && averageG<160 ){
      colour = 2;
      played=0;
      Serial.println("RED");           
       OCR1AL = 255;
        delay(400);
       OCR1AL = 196;
        delay(400);
    }

    else {
      played=0;
      Serial.println("NOTHING");
      colour = 0;
      OCR1AL=0;

    }
    
    averageR=0;
    averageG=0;
    averageB=0;
    
    TotalR=0;
    TotalG=0;
    TotalB=0;
  }
  delay(10);
     stop = 1;
//      Serial.print("Distance: ");
//      Serial.print(distance);
//      Serial.println(" cm"); 

//     if(colour == 0 ) {
////      Serial.println("Undefined");
//      OCR1AL=0;
//      delay(1000);
//     }
//     else if(colour ==1){
////        Serial.println("COLOUR: GREEN");
//         OCR1AL = 200;
//         delay(1000);
//         OCR1AL = 0;
//     }
//     else if(colour == 2) {
////        Serial.println("COLOUR: RED");
//        OCR1AL = 255;
//        delay(400);
//        OCR1AL = 196;
//        delay(400);
//     }
//      
     
  }
}
