

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
#include <avr/pgmspace.h>



uint8_t analogChannelRead=1;
uint16_t analogValues[3];
uint16_t lastAnalogValues[3];



uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap){
  uint32_t inMin=0, inMax=255, outMin=0, outMax=255;
  for(int i=0;i<numberOfPoints-1;i++){
    if(value >= tableMap[i] && value <= tableMap[i+1]) {
      inMax=tableMap[i+1];
      inMin=tableMap[i];
      outMax=tableMap[numberOfPoints+i+1];
      outMin=tableMap[numberOfPoints+i];
      i=numberOfPoints+10;
    }
  }
  return map(value,inMin,inMax,outMin,outMax);
}


void setup()  { 
  setTimers(); //setup audiorate interrupt

  pinMode(0, OUTPUT);
  // digitalWrite(0,LOW);
  pinMode(1, OUTPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  //  pinMode(1, INPUT_PULLUP);
  // digitalWrite(1,LOW);
  //  analogRead(A3);

  init();
  connectChannel(analogChannelRead);
  startConversion();

} 

void setTimers(void)
{
  /*
  TCCR0A=0;
   TCCR0B=0;
   bitWrite(TCCR0A,COM0A0,0);
   bitWrite(TCCR0A,COM0A1,1);
   bitWrite(TCCR0A,COM0B0,0);
   bitWrite(TCCR0A,COM0B1,1);
   bitWrite(TCCR0A,WGM00,1);
   bitWrite(TCCR0A,WGM01,1);
   bitWrite(TCCR0B,WGM02,0);
   bitWrite(TCCR0B,CS00,1);
   */
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS11);//  | _BV(CS11) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  bitWrite(TCCR1,CS12,0);
  sei();
}


uint8_t analogPins[3]={
  A1,A2,A3};

uint8_t _xor;
int _val;
uint16_t index;
uint8_t decayVolume;
int out,in;

uint8_t analogReadChannels[4]={
  1,2,1,3};
void loop() { 
  // WAVE_TABLES[0];

  if(isConversionFinished()){
    lastAnalogValues[analogReadChannels[analogChannelRead]-1]=analogValues[analogReadChannels[analogChannelRead]-1];
    analogValues[analogReadChannels[analogChannelRead]-1]= getConversionResult();
    analogChannelRead++;
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogReadChannels[analogChannelRead]);
    startConversion();
  }

  if( lastAnalogValues[2]!= analogValues[2]) setFrequency(analogValues[2]);
  if( lastAnalogValues[1]!= analogValues[1])  setAmplification(analogValues[1]);
  //if( lastAnalogValues[0]!= analogValues[0]) setLength(analogValues[0]);
_delay_us(200);

}


void setFrequency(int _freq){
  _freq=1024-_freq;
  uint8_t preScaler=_freq>>7;
  preScaler+=2; //*2
  for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 

  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  OCR1A=compare+128; 
}
uint8_t amp;
void setAmplification(int _amp){
  amp=_amp>>2;
}

//#define SIZE_OF_BUFFER 450
//uint8_t buffer[450];
int lastIndex;
bool flop;
ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
 // flop=!flop;
//  bitWrite(PINB,PB1,flop);
  OCR0A=constrain(((analogValues[0]>>2)*amp)>>8,0,255);
  TCNT1 = 0; 
}











