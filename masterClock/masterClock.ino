

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
uint8_t analogChannelRead=3;
uint16_t analogValues[4];
uint16_t lastAnalogValues[4];
void setup()  { 
  setTimers(); //setup audiorate interrupt

  pinMode(0, OUTPUT);
  //  pinMode(A1, INPUT);
  // pinMode(A2, INPUT);
  //pinMode(A3, INPUT);
  pinMode(1, OUTPUT);
  pinMode(2, INPUT); //change for final
  pinMode(3, INPUT); 
  pinMode(4, INPUT);
  digitalWrite(0,LOW);
  digitalWrite(1,LOW);
  // digitalWrite(2,LOW);
  //  analogRead(A3);

  init();
  connectChannel(analogChannelRead);
  startConversion();

} 

void setTimers(void)
{

  // TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  // TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS11);//| _BV(CS12) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  sei();
}

uint8_t clkCounter=0,rstCounter=0;
#define TRIGGER_LENGTH 100
bool butState;
bool play;
uint32_t tempoCounter=0;
bool usePin[4]={true,false,true,true};
void loop() { 

  if(isConversionFinished()){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    analogChannelRead++;
    while(!usePin[analogChannelRead]){
      analogChannelRead++;
      if(analogChannelRead>3) analogChannelRead=0;
    }
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogChannelRead);
    startConversion();
  }


  setTempo(analogValues[3]);//analogRead(A3));
  setShuffle(analogValues[2]);//analogRead(A2)>>2);
  //  if( lastAnalogValues[0]!= analogValues[0])_xor=analogValues[0]>>2;//analogRead(A1)>>2;
 // bitWrite(PORTB,PB2,1);
  bool newState=bitRead(PINB,PINB2);
  if(!butState &&  newState) {
    if(play) play=false;
    else  play=true, rstCounter=0, clkCounter=0,tempoCounter=0;
  }
  butState=newState;
  _delay_ms(1);
}
uint32_t tempoThreshold;
uint32_t shuffleThreshold;
void setTempo(uint16_t _tempo){
  tempoThreshold=map(_tempo,0,1024,20000,TRIGGER_LENGTH *2);
}
void setShuffle(uint16_t _shuffle){
  shuffleThreshold=map(_shuffle,0,1024,tempoThreshold/2,(tempoThreshold/4)*3);
}
void _digitalWrite(uint8_t _state){
  bitWrite(PORTB,PB1,_state);
  //if(_state==0) PORTB &= ~(1<<PB1);
  // else  PORTB |= 1<<PB1;
}



ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  //  pinMode(2, INPUT_PULLUP);


  if(play){
    tempoCounter++;
    if(tempoCounter==shuffleThreshold) clkCounter=0;
    if(tempoCounter>=tempoThreshold) clkCounter=0,tempoCounter=0;
    if(clkCounter<TRIGGER_LENGTH) bitWrite(PORTB,PB0,1), clkCounter++;
    else bitWrite(PORTB,PB0,0);
    if(rstCounter<TRIGGER_LENGTH) bitWrite(PORTB,PB1,1), rstCounter++;
    else bitWrite(PORTB,PB1,0);
  }
  else{
    bitWrite(PORTB,PB0,0);
    bitWrite(PORTB,PB1,0);
  }

}






