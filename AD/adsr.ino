
//#include <math.h>



enum envState {

};

const uint8_t  env_idle = 0;
const uint8_t env_attack=1;
const uint8_t env_decay=2;
const uint8_t env_sustain=3;
const uint8_t env_release=4;
const uint8_t env_hold=5;
bool gateState;
bool hold;
bool looping;
int state;
long hold_count;
long hold_value;
float output;
float attackRate;
float decayRate;
float releaseRate;
float attackCoef;
float decayCoef;
float releaseCoef;
float sustainLevel;
float targetRatioA;
float targetRatioDR;
float attackBase;
float decayBase;
float releaseBase;



float process() {
  switch (state) {
  case env_hold:
    hold_count++;
    if(hold_count>=hold_value){
      hold_count=0;
      if(looping) state = env_release;
      else state = env_decay;
    }

    output = 1.0;
    break;
  case env_idle:
    if(looping){
      state = env_attack;
    }
    break;
  case env_attack:
    output = attackBase + output * attackCoef;
    if (output >= 1.0) {
      output = 1.0;
      if(hold) state = env_hold, hold_count=0;
      else if(looping) state = env_release;
      else state = env_decay;
    }
    break;
  case env_decay:
    output = decayBase + output * decayCoef;
    if (output <= sustainLevel) {
      output = sustainLevel;
      state = env_sustain;
    }
    break;
  case env_sustain:
    if(sustainLevel<=0.0) state = env_idle;
    break;
  case env_release:
    output = releaseBase + output * releaseCoef;
    if (output <= 0.0) {
      output = 0.0;
      if(looping) state = env_attack;
      else state = env_idle;
    }
  }
  return output;
}

void gate(int gate) {
  if (gate) gateState=true, state = env_attack;
  else if (state != env_idle){
    if(!hold){
      state = env_release;
      gateState=false;
    }
  }
}

int getState() {
  return state;
}

void reset() {
  state = env_idle;
  output = 0.0;
}

float getOutput() {
  return output;
}




void setHoldRate(long rate){ 
  hold_value=rate;
};
void setLoop(bool _loop){ 
  looping=_loop;
};
void sync(){

}
void setHold(bool _hold){
  hold=_hold;
  if(hold) setSustainLevel(0);
};
void reTrigger(){
  if(gateState) state=env_attack;
}
void setAttackRate(float rate) {
  attackRate = rate;
  attackCoef = calcCoef(rate, targetRatioA);
  attackBase = (1.0 + targetRatioA) * (1.0 - attackCoef);
}

void setDecayRate(float rate) {
  decayRate = rate;
  decayCoef = calcCoef(rate, targetRatioDR);
  decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
}

void setReleaseRate(float rate) {
  releaseRate = rate;
  releaseCoef = calcCoef(rate, targetRatioDR);
  releaseBase = -targetRatioDR * (1.0 - releaseCoef);
}

float calcCoef(float rate, float targetRatio) {
  return exp(-log((1.0 + targetRatio) / targetRatio) / rate);
}

void setSustainLevel(float level) {
  sustainLevel = level;
  decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
}

void setTargetRatioA(float targetRatio) {
  if (targetRatio < 0.000000001)
    targetRatio = 0.000000001;  // -180 dB
  targetRatioA = targetRatio;
  attackBase = (1.0 + targetRatioA) * (1.0 - attackCoef);
}

void setTargetRatioDR(float targetRatio) {
  if (targetRatio < 0.000000001)
    targetRatio = 0.000000001;  // -180 dB
  targetRatioDR = targetRatio;
  decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
  releaseBase = -targetRatioDR * (1.0 - releaseCoef);
}


