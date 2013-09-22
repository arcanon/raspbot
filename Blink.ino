/*
  This listens for SPI commands from the raspberry pi and drives
  2 DC motors via a L293 H bridge.
 */
#include <SPI.h>
 
const byte SETTING_DBG_OUT = 0x1;
const int spiBufLen = 4;
const int ringBufferSize = 20;
const int totalBufLen = spiBufLen*ringBufferSize;
// Readbyte 0 is the direction or action, readbyte 1+2 are the right and left
// speed
volatile byte readBytes[ringBufferSize*spiBufLen];
volatile byte writeBytes[ringBufferSize*spiBufLen];
volatile int currentReadPos = 0;
volatile int absoluteReadPos = 0;
volatile int currentWritePos = 0;
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

byte speedLoop=0;
byte rightOn = 0;
byte leftOn = 0;
byte toggleOn = 1;

byte rightSpeed = 255;
byte leftSpeed = 255;

void enableRightForwards()
{
  digitalWrite(rightEnable, LOW);
  digitalWrite(rightBackwards, LOW);
  digitalWrite(rightForwards, HIGH);
  digitalWrite(rightEnable, HIGH);
  rightOn = 1;
  rightSpeed = readBytes[currentReadPos+1];
  Serial.println("enableRightForwards");
}

void enableRightBackwards()
{
  digitalWrite(rightEnable, LOW);
  digitalWrite(rightBackwards, HIGH);
  digitalWrite(rightForwards, LOW);
  digitalWrite(rightEnable, HIGH);
  rightOn = 1;
  rightSpeed = readBytes[currentReadPos+1];
  Serial.println("enableRightBackwards");
}

void enableLeftForwards()
{
  digitalWrite(leftEnable, LOW);
  digitalWrite(leftBackwards, LOW);
  digitalWrite(leftForwards, HIGH);
  digitalWrite(leftEnable, HIGH);
  leftOn = 1;
  leftSpeed = readBytes[currentReadPos+2];
  Serial.println("enableLeftForwards");
}

void enableLeftBackwards()
{
  digitalWrite(leftEnable, LOW);
  digitalWrite(leftBackwards, HIGH);
  digitalWrite(leftForwards, LOW);
  digitalWrite(leftEnable, HIGH);
  leftOn = 1;
  leftSpeed = readBytes[currentReadPos+2];
  Serial.println("enableLeftBackwards");
}

void disableRight()
{
  digitalWrite(leftEnable, LOW);
  rightOn = 0;
  Serial.println("disableRight");
}
void disableLeft()
{
  digitalWrite(rightEnable, LOW);
  leftOn = 0;
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
  
  for (int i = 0; i < ringBufferSize; i++)
  {
    readBytes[i*spiBufLen] = 0;
    readBytes[i*spiBufLen+1] = 255;
    readBytes[i*spiBufLen+2] = 255;
    readBytes[i*spiBufLen+3] = SETTING_DBG_OUT;
    writeBytes[i*spiBufLen] = 0xcd;
    writeBytes[i*spiBufLen+1] = 0xcd;
    writeBytes[i*spiBufLen+2] = 0xcd;
    writeBytes[i*spiBufLen+3] = 0xcd;
  }
}

ISR (SPI_STC_vect)
{
  // We need to setup the next byte to be read
  SPDR = (byte)writeBytes[(intCount+1) % (totalBufLen)];
    
  readBytes[intCount % totalBufLen] = SPDR;  // grab byte from SPI Data Register
  
  if (intCount % spiBufLen == 0)
  {
    // debug info
    writeBytes[(intCount+3) % totalBufLen] = readBytes[intCount % totalBufLen];  
  }
  intCount++;
}  // end of interrupt routine SPI_STC_vect

// the loop routine runs over and ovier again forever:
void loop() 
{
  int V0;
  const int ThermistorPin = 3; // Analog input pin for thermistor voltage
  const int VoltageMonitor = 2; // Analog input pin for thermistor voltage
  int Vo; // Integer value of voltage reading
  float R = 4000.0; // Fixed resistance in the voltage divider
  float logRt,Rt,T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  Vo = analogRead(VoltageMonitor);
  int currPos = (intCount%totalBufLen)/spiBufLen;
  writeBytes[currPos+1] = Vo&0xff;
  writeBytes[currPos+2] = (Vo>>8)&0xff;

  if (speedLoop < rightSpeed)
  {
    // enable outputs
    if (rightOn)
    {
      digitalWrite(rightEnable, HIGH);
    }
    else
    {
      digitalWrite(rightEnable, LOW);
    }
  }
  else
  {
    digitalWrite(rightEnable, LOW);
  }
  
  if (speedLoop < leftSpeed)
  {
    if (leftOn)
    {
      digitalWrite(leftEnable, HIGH);
    }
    else
    {
      digitalWrite(leftEnable, LOW);
    }
  } 
  else
  {
    digitalWrite(leftEnable, LOW);
  }
  
  speedLoop++;
  
  if ((absoluteReadPos <= intCount-4) ||
      (intCount < absoluteReadPos)) {
    switch(readBytes[currentReadPos+0])
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
    readBytes[currentReadPos+0] = 0;
    currentReadPos = (currentReadPos+spiBufLen) % totalBufLen;
    absoluteReadPos = absoluteReadPos + spiBufLen;
  }
  
  now=millis();
  
  // Debug output, we don't want to do this to often because
  // its slow
  if ((readBytes[currentReadPos+3] & SETTING_DBG_OUT) &&
      (now - lastprint > 500))
  {
    Serial.print(Vo);
    Serial.print(" spcr ");
    Serial.print(SPCR, BIN);
    //Serial.print(" spsr ");     
    //Serial.print(SPSR, BIN);
    Serial.print(" readByte "); 
    Serial.print(readBytes[currentReadPos + 0]);
    Serial.print(" intC ");
    Serial.print(intCount);
    Serial.print(" speed "); 
    Serial.print(readBytes[currentReadPos + 1]);
    Serial.print(" reapdpos ");
    Serial.print(absoluteReadPos);
    //Serial.print(" wb1 ");
    //Serial.print(writeBytes[2]);
    Serial.println(" ");
    lastprint = now;    
  }
}
