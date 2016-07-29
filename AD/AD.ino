

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
//#include <EEPROM.h>
#include <avr/interrupt.h>
//#include <expADSR.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
//#include <Oscil.h>
// Only use Serial if using ATTiny85
// Serial output connections:
//#include <SoftwareSerial.h>
//#define rxPin 5    // We use a non-existant pin as we are not interested in receiving data
//#define txPin 3
//SoftwareSerial serial(rxPin, txPin);
uint8_t analogChannelRead=1;
uint16_t analogValues[4];
uint16_t lastAnalogValues[4];
//ADSR envelope;
#define ADSRMAP_POINTS 5
uint16_t ADSRMap[10]={
  0,63,127,191,255,  50,1500,3000,15000,50000};
  uint16_t ADSRMap3[10]={
  0,63,127,191,255,  5,1500,3000,15000,50000};
uint16_t ADSRMap2[10]={
  0,63,127,210,255,  5,100,500,1000,20000};
bool readADC[4]={true,true,false,false};

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
  pinMode(1, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(5,HIGH);
  //  pinMode(1, INPUT_PULLUP);
  // digitalWrite(1,LOW);
  //  analogRead(A3);

  init();
  connectChannel(analogChannelRead);
  startConversion();
  setTargetRatioA(0.5); //100 for linear
  setTargetRatioDR(0.3); //100 for linear
  setSustainLevel(0.0);

  setAttackRate(curveMap(100,ADSRMAP_POINTS,ADSRMap));
  setDecayRate(curveMap(100,ADSRMAP_POINTS,ADSRMap));
  setReleaseRate(curveMap(100,ADSRMAP_POINTS,ADSRMap));
  setHoldRate(curveMap(0,ADSRMAP_POINTS,ADSRMap));
  setLoop(false);

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
  TIMSK = _BV(OCIE1A) ;//| _BV(TOIE0);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12) ;//| _BV(CS11);//  | _BV(CS11) ;// //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  sei();
}


uint8_t analogPins[3]={
  A1,A2,A3};

uint8_t _xor;
int _val;
bool _gate;
int out,in;
bool render;
bool slewMode;
bool cycle;
void loop() { 
  // bitRead(PINB,PINB1);
  if(isConversionFinished()){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    analogChannelRead++;
    /*
    while(readADC[analogChannelRead]){
      analogChannelRead++;
      if(analogChannelRead>3) analogChannelRead=1;
    }
    */
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogChannelRead);
    startConversion();
  }

  if(cycle){
    if( lastAnalogValues[3]!= analogValues[3]) setAttackRate(curveMap(analogValues[3]>>2,ADSRMAP_POINTS,ADSRMap2));//analogRead(A3));
    if( lastAnalogValues[2]!= analogValues[2]){
      setDecayRate(curveMap(analogValues[2]>>2,ADSRMAP_POINTS,ADSRMap2));
      setReleaseRate(curveMap(analogValues[2]>>2,ADSRMAP_POINTS,ADSRMap2));
    }//analogRead(A2)>>2);
  }
  else{
    if( lastAnalogValues[3]!= analogValues[3]) setAttackRate(curveMap(analogValues[3]>>2,ADSRMAP_POINTS,ADSRMap));//analogRead(A3));
    if( lastAnalogValues[2]!= analogValues[2]){
      setDecayRate(curveMap(analogValues[2]>>2,ADSRMAP_POINTS,ADSRMap3));
      setReleaseRate(curveMap(analogValues[2]>>2,ADSRMAP_POINTS,ADSRMap));
    }
  }

  cycle=bitRead(PINB,PINB1);
  //cycle=(analogValues[0]>900);
  if(analogValues[0]>900) {
    if(slewMode) reset();
    slewMode=false;
    render=true;
  }
  else slewMode=true;
//  cycle=true;
  if(cycle){
    render=true;
    setLoop(true);
    if( lastAnalogValues[1]!= analogValues[1]){
      in=analogValues[1]>>2;
      if(lastAnalogValues[1]<500 && analogValues[1]>=500) reset();
    }
  }
  else{
    setLoop(false);
    if( lastAnalogValues[1]!= analogValues[1]){
      in=analogValues[1]>>2;
      //if(in>100) _gate=true, render=true;
      //else _gate=false, render=true;
      if(slewMode){
        if(in>out) _gate=true, render=true;
        if(in<out) _gate =false, render=true;
      }
    };
    gate(_gate);
  }
  //analogRead(A1)>>2;
  _delay_us(250);
  // uint8_t freq=analogRead(A3)>>2;
  //_div=(analogRead(A2)>>)+1;
  //OCR1A=constrain(freq,1,255); //speed of audiorate interrupt

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

uint8_t _time;
ISR(TIMER0_OVF)  //audiorate interrupt TOIE0
{
_time++;
}

uint16_t interuptis(){
 return word(_time,TCNT0); 
}

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  if(render){
    float  fout=(float)255*process();
    out=(int)(fout);
  }

  if(!cycle){
    if(slewMode){
      if(_gate){
        if(out>=in) out=in,render=false;
      }
      else {
        if(out<=in) out=in, render=false;
      }
    }
    else{
      render=true;
     if(in>100) _gate=true;
     else _gate=false; 
    }
  }
  OCR0A= out;//constrain(renderVco()^_xor,0,255); //pwm output
  TCNT1 = 0; 
}





