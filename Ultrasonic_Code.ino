// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define trigPin 18 // PB4, A4
#define echoPin 19 // PB5, A5

// defines variables
long duration = 0; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
unsigned long StartTime;
unsigned long CurrentTime;
bool stop = 0;
bool flag = false;

void setup() {
  // Sets the trigPin (PB4) as an Output
  DDRC |= 1 << 4;
  /// Sets the echoPin (PB5) as Input
  DDRC |= 0 << 5;
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}

void loop() {
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
  
  if (stop == 0)
  {
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm"); 
  }
  
  if (distance <= 13)
  {
     stop = 1;
  }
}
