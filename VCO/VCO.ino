

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
//#include <Oscil.h>
// Only use Serial if using ATTiny85
// Serial output connections:
//#include <SoftwareSerial.h>
#define rxPin 5    // We use a non-existant pin as we are not interested in receiving data
#define txPin 3
//SoftwareSerial serial(rxPin, txPin);
uint8_t analogChannelRead=1;
uint16_t analogValues[4];
uint16_t lastAnalogValues[4];
void setup()  { 
  setTimers(); //setup audiorate interrupt

  pinMode(0, OUTPUT);
   pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(1, OUTPUT);
  digitalWrite(1,LOW);
   digitalWrite(5,LOW);
 //  analogRead(A3);
  
  init();
  connectChannel(analogChannelRead);
  startConversion();
  setWaveshape(0);
  setFrequency(0);
 // xor=0;
} 

void setTimers(void)
{

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
  TCCR1 = _BV(CTC1) | _BV(CS12) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  sei();
}


uint8_t analogPins[3]={
  A1,A2,A3};
  
uint8_t _xor;
int _val;
bool quantizer;
void loop() { 
 // bitRead(PINB,PINB1);
  if(isConversionFinished()){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead]; //-1
     analogValues[analogChannelRead]= getConversionResult();
     analogChannelRead++;
     if(analogChannelRead>3) analogChannelRead=0; //=1;
     connectChannel(analogChannelRead);
     startConversion();
   }
   
  
  if( lastAnalogValues[1]!= analogValues[1]) _xor=255-(analogValues[1]>>2);//analogRead(A3)); 1
 if( lastAnalogValues[3]!= analogValues[3]) setFrequency(analogValues[3]);//analogRead(A2)>>2); 0
  if( lastAnalogValues[2]!= analogValues[2]) setWaveshape(analogValues[2]>>2); //analogRead(A1)>>2; 2
  _delay_us(250);
  // uint8_t freq=analogRead(A3)>>2;
  //_div=(analogRead(A2)>>)+1;
  //OCR1A=constrain(freq,1,255); //speed of audiorate interrupt
 if(analogValues[0]>900) quantizer=false; 
  else quantizer=true;
}
void _digitalWrite(uint8_t _state){
  bitWrite(PORTB,PB1,_state);
  //if(_state==0) PORTB &= ~(1<<PB1);
  // else  PORTB |= 1<<PB1;
}

int pwmCounter;

uint16_t upIncrement=0;
uint16_t downIncrement=255;
uint32_t _upIncrement=0;
uint32_t _downIncrement=255;
uint8_t pwmIncrement;
uint8_t waveshape,lastWaveshape;

void _setFrequency(int _freq){
  if(_freq>768) pwmIncrement=8;
  else if(_freq>512) pwmIncrement=4;
  else if(_freq>256) pwmIncrement=2;
  else pwmIncrement=1;
  _upIncrement=pwmIncrement*upIncrement;
   _downIncrement=pwmIncrement*downIncrement;
  uint8_t _FRR=_freq;
  _FRR=map(_FRR,0,255,255,95);
  //_FRR/=2;
  OCR1A=constrain(_FRR,1,255);
}

uint8_t tuneTable[12]={0,/*11,*/21,/*32,*/42,/*53,*/63,/*74,*/84,/*95,*/106/*,116*/};
void setFrequency(int _freq){
  _freq=1024-_freq;
  uint8_t preScaler=_freq>>7;
  preScaler+=3; //*2
  pwmIncrement=4;
 /*
  if(preScaler<5){
    if(preScaler==3) pwmIncrement=4;
    if(preScaler==4) pwmIncrement=2;
   preScaler=3; 
  }
  else {
    preScaler-=2;
    pwmIncrement=1;
  }
  */
 /*
  if(preScaler<4){
   pwmIncrement=1<<preScaler;
   preScaler=1;
  }
  else{
    pwmIncrement=1;
    preScaler-=3; 
  }
  */
   
  _upIncrement=pwmIncrement*upIncrement;
   _downIncrement=pwmIncrement*downIncrement;
  for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 
  
  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  if(quantizer){
    //OCR1A=tuneTable[map(compare,0,127,0,12)];
    compare=tuneTable[map(compare,0,127,0,6)];//map(map(compare,0,127,0,12),0,12,0,127);
  }
  OCR1A=compare+128; 
// OCR1A=compare<<1; 
  
  
 // OCR1A=tuneTable[map(compare,0,127,0,12)]; //quantized
  
}



void setWaveshape(uint8_t _waveshape){
  lastWaveshape=waveshape;
  waveshape=constrain(_waveshape,2,254);
  if(lastWaveshape!=waveshape){
    upIncrement=65535/waveshape;
    downIncrement=65535/(256-waveshape);

  }
}

long _value;
bool goingUp;
uint16_t counter;

//divider++;
// if(dividerCounter
/*
  switch(divider){
 case 0:
 if(counter>127) counter=0;
 break; 
 case 1:
 break;
 }
 */
//render square output


uint8_t renderVco(){
  //render waveshape output
 // for(uint8_t i=0;i<2;i++){
    counter+=pwmIncrement;  
    pwmCounter+=pwmIncrement;
    if(goingUp)  _value+=_upIncrement;
    else _value-=_downIncrement;
    //if(pwmCounter<waveshape) goingUp=true;
   //else goingUp=false;
    if(_value>=65535) _value=65535, goingUp=false; //65535 65535
   if(_value<=0) _value=0;
   if(counter>=255) counter=0;
    if(counter==0) pwmCounter=0,goingUp=true;
 // }
 return _value>>8;
}

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
    TCNT1 = 0; 
  OCR0A= constrain(255-(renderVco()|_xor),0,255); //pwm output
 // OCR0A= constrain(renderVco()^_xor,0,255);
 OCR0B=0;
  if(pwmCounter<waveshape)  bitWrite(PORTB,PB1,true);//, OCR0B=0;//bitWrite(PORTB,PB1,true);
  else bitWrite(PORTB,PB1,false); //OCR0B=0,

}

