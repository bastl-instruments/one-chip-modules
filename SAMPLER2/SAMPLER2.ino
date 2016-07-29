

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
#include <avr/pgmspace.h>

//uncomment only ONE of these to select sample

//#include <CB_AT.h>
//#include <CB2_AT.h>
//#include <CB3_AT.h>
#include <CB4_AT.h>
//#include <GB_BLIP_AT.h>
//#include <GB_BZZ_AT.h>
//#include <GB_GLITCH_AT.h>
//#include <GB_HH_AT.h>
//#include <GB_KICK_AT.h>
//#include <GB_SNARE_AT.h>
//#include <GB_TUI_AT.h>
//#include <GL_A_AT.h>
//#include <GL_B_AT.h>
//#include <GL_C_AT.h>
//#include <GL_D_AT.h>
//#include <GL_E_AT.h>
//#include <GL_F_AT.h>
//#include <GL_G_AT.h>
//#include <GL_H_AT.h>
//#include <GL_I_AT.h>
//#include <HAT_AT.h>
//#include <HAT2_AT.h>
//#include <KICK_AT.h>
//#include <KICK2_AT.h>
//#include <RIDE_AT.h>
//#include <SNARE_AT.h>
//#include <SNARE2_AT.h>
//#include <TR_CB_AT.h>
//#include <TR_CLAP_AT.h>
//#include <TR_HH_AT.h>
//#include <TR_KICK_AT.h>
//#include <TR_OH_AT.h>
//#include <TR_RIM_AT.h>
//#include <TR_SNARE_AT.h>
//#include <TR_TOM_AT.h>


uint8_t analogChannelRead=1;
uint16_t analogValues[4];
uint16_t lastAnalogValues[4];



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
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  // digitalWrite(1,LOW);
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
bool render;
bool cycle;
bool gate;
uint16_t decayCounter, decayTime;
uint8_t debounceCounter;
bool fadeIn;
uint16_t sampleStart=0;
void loop() { 
  // WAVE_TABLES[0];

  if(isConversionFinished()){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    analogChannelRead++;
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogChannelRead);
    startConversion();
  }


  if( lastAnalogValues[3]!= analogValues[3]) setFrequency(analogValues[3]);
  if( lastAnalogValues[1]!= analogValues[1]) setDecay(analogValues[1]);
  if( lastAnalogValues[2]!= analogValues[2]) setLength(analogValues[2]);
  if( lastAnalogValues[0]!= analogValues[0]) setStart(analogValues[0]);
  
  debounceCounter++;
  if(debounceCounter>3){
    debounceCounter=0;
    bool newState=bitRead(PINB,PINB1);
    if(newState && !gate){
      decayVolume=255, index=0, render=true, fadeIn=true; 
    }
    gate=newState;
  }
  if(decayTime!=0){
    if(!gate){
      decayCounter+=8;
      if(decayCounter>=decayTime)
      {
        decayCounter=0;
        if(decayVolume>0) decayVolume--;
      }
    }
  }
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

void setDecay(int _decay){
  decayTime=_decay>>2;
}
uint16_t sampleEnd;
void setLength(int _length){
  sampleEnd=map(_length,0,1024,8,sampleLength);
}
void setStart(int _start){
 sampleStart=0;//constrain(map(analogValues[0],750,1000,sampleLength,0),sampleLength,0);
 
}

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  if(render) index++;
  if(index>=sampleEnd){
    if(decayTime==0) {
      render=false;
    }
    else render=true;
    index=sampleStart;
  }
 // if(decayVolume==0) out=0;
//  else 
/*
  if(fadeIn){
    if(index==0){
      out = (char)pgm_read_byte_near(sampleTable+index)+128;
      out=(out*50)>>8;
    }
    else if(index==1){
      out = (char)pgm_read_byte_near(sampleTable+index)+128;
      out=(out*100)>>8;
    }
    else if(index==2){
      out = (char)pgm_read_byte_near(sampleTable+index)+128;
      out=(out*200)>>8;
      fadeIn=false;
    }
  }
  else{
    */
    out = (char)pgm_read_byte_near(sampleTable+index);
    out=(out*decayVolume)>>8;
 // }
 OCR0A= out+128;
 // else OCR0A=0;//constrain(out,0,255);//constrain(renderVco()^_xor,0,255); //pwm output
  // OCR0B= runglerOut;
  TCNT1 = 0; 
}








