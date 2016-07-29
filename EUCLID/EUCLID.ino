

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
#include <euclid.h>
euclid euc;
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
  //  pinMode(A1, INPUT);
  // pinMode(A2, INPUT);
  //pinMode(A3, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT); //change for final
  pinMode(4, INPUT); //change for final
  digitalWrite(0,LOW);

  //  analogRead(A3);

  //  init();
  //connectChannel(analogChannelRead);
  //startConversion();
  euc.generateSequence(16,31);
  init();
  connectChannel(analogChannelRead);
  startConversion();

} 

void setTimers(void)
{

  // TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  // TCCR0B = 0<<WGM02 | 1<<CS00;
  OCR0A=0;
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


uint8_t analogPins[3]={
  A1,A2,A3};

uint8_t _xor;
int _val;
uint8_t counter;
bool clkState;
bool rstState;
uint8_t address=0;
uint8_t steps,fills, lastSteps, lastFills;
bool butState;
uint8_t clkCounter;
bool usePin[4]={
  true,false,true,true};
void loop() { 
  if(isConversionFinished()){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    analogChannelRead++;
    
    while(!usePin[analogChannelRead]){
     analogChannelRead++;
     if(analogChannelRead>3) analogChannelRead=0;
     }
     
    // analogChannelRead=
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogChannelRead);
    startConversion();
  }
  steps=map(analogValues[3],0,1024,1,32);
  fills=map(analogValues[2],0,1024,1,steps);


  if(steps !=lastSteps){

    if(analogValues[0]>900) euc.generateSequence(steps,fills);
    else euc.generateRandomSequence(steps,fills);
  }
  lastSteps=steps;



  if(fills!=lastFills){

    if(analogValues[0]>900) euc.generateSequence(steps,fills);
    else euc.generateRandomSequence(steps,fills);
  }
  lastFills=fills;

  //  newState=bitRead(PINB,PINB4);

  bool newState=bitRead(PINB,PINB1);
  if(!butState &&  newState) {
    euc.doStep();
    if(euc.getCurrentStep())  clkCounter=0;
    // clkCounter=0;
  }
  butState=newState;

  _delay_us(250);

}
void _digitalWrite(uint8_t _state){
  bitWrite(PORTB,PB1,_state);
  //if(_state==0) PORTB &= ~(1<<PB1);
  // else  PORTB |= 1<<PB1;
}

//uint16_t counter;

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


#define TRIGGER_LENGTH 100

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  if(clkCounter<TRIGGER_LENGTH) bitWrite(PORTB,PB0,1), clkCounter++;
  else bitWrite(PORTB,PB0,0);
}











