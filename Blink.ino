/*
  This listens for SPI commands from the raspberry pi and drives
  2 DC motors via a L293 H bridge.
 */
#include <SPI.h>
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 8;
volatile boolean readTemp = true;
volatile byte readByte = 0;
byte clr;
int intCount = 0;
const int leftEnable = 5;
const int leftForwards = 6;
const int leftBackwards = 7;
const int rightEnable = 4;
const int rightForwards = 3;
const int rightBackwards = 2;
const int leftStop = 15;
const int rightStop = 16;

void enableRightForwards()
{
  digitalWrite(rightEnable, LOW);
  digitalWrite(rightBackwards, LOW);
  digitalWrite(rightForwards, HIGH);
  digitalWrite(rightEnable, HIGH);
  Serial.println("enableRightForwards");
}

void enableRightBackwards()
{
  digitalWrite(rightEnable, LOW);
  digitalWrite(rightBackwards, HIGH);
  digitalWrite(rightForwards, LOW);
  digitalWrite(rightEnable, HIGH);
  Serial.println("enableRightBackwards");
}

void enableLeftForwards()
{
  digitalWrite(leftEnable, LOW);
  digitalWrite(leftBackwards, LOW);
  digitalWrite(leftForwards, HIGH);
  digitalWrite(leftEnable, HIGH);
  Serial.println("enableLeftForwards");
}

void enableLeftBackwards()
{
  digitalWrite(leftEnable, LOW);
  digitalWrite(leftBackwards, HIGH);
  digitalWrite(leftForwards, LOW);
  digitalWrite(leftEnable, HIGH);
  Serial.println("enableLeftBackwards");
}

void disableRight()
{
  digitalWrite(leftEnable, LOW);
  Serial.println("disableRight");
}
void disableLeft()
{
  digitalWrite(rightEnable, LOW);
  Serial.println("disableLeft");
}

  unsigned long lastprint, now;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  //pinMode(led, OUTPUT); 
  pinMode(leftEnable, OUTPUT);   
  pinMode(rightEnable, OUTPUT);   
  pinMode(leftForwards, OUTPUT);   
  pinMode(leftBackwards, OUTPUT);   
  pinMode(rightForwards, OUTPUT);   
  pinMode(rightBackwards, OUTPUT);   
  Serial.begin(9600); 
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE); 
  SPI.attachInterrupt();
  clr=SPSR;
  clr=SPDR;
  delay(10);
  disableRight();
  disableLeft();
  
  lastprint = millis();
  Serial.println("setup done");
  
}


ISR (SPI_STC_vect)
{
  readByte = SPDR;  // grab byte from SPI Data Register
  //Serial.print("Interrupt. ");
  //Serial.print(readByte);
  //Serial.println(" ");
  // add to buffer if room
  //if (readByte == 0x5)
  {
    readTemp = !readTemp;  
      
  }  // end of room available
  
  
 
  //readByte = 0;
  intCount++;
  //readTemp = false;
}  // end of interrupt routine SPI_STC_vect

// the loop routine runs over and over again forever:
void loop() {
  int V0;
  int ThermistorPin = 3; // Analog input pin for thermistor voltage
  int Vo; // Integer value of voltage reading
  float R = 4000.0; // Fixed resistance in the voltage divider
  float logRt,Rt,T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  Vo = analogRead(ThermistorPin);

  //Rt = R*( 1023.0 / (float)Vo - 1.0 );
  //logRt = log(Rt);
  //T = ( 1.0 / (c1 + c2*logRt + c3*logRt*logRt*logRt ) ) - 273.15;
  
  //digitalWrUite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //enableLeftBackwards();
  
  switch(readByte)
  {
    case leftForwards:
      enableLeftForwards();
      break;
    case leftBackwards:
      enableLeftBackwards();
      break;
    case rightForwards:
      enableRightForwards();
      break;
    case rightBackwards:
      enableRightBackwards();
      break;    
    case rightStop:
      disableRight();
      break;
    case leftStop:
      disableLeft();
      break;
  }
  readByte = 0;
  
  now=millis();
  //delay(100);               // wait for a second
  //if (readTemp) 
  //  Serial.print("interrupt called ");
   
  if (now - lastprint > 500)
  {
    Serial.print(Vo);
    Serial.print(" spcr ");
    Serial.print(SPCR, BIN);
    Serial.print(" spsr ");     
    Serial.print(SPSR, BIN);
    Serial.print(" readByte "); 
    Serial.print(readByte);
    Serial.print(" intC ");
    Serial.print(intCount); 
    Serial.println(" ");
    lastprint = now;    
  }
  

  //
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(100);               // wait for a second
}
