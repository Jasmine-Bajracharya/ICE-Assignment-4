#include <Servo.h>

const uint8_t PIN[6] = {3, 5, 6, 10, 11, 9}; // base, shoulder, elbow, wristPitch, wristRoll, gripper
int8_t  DIR[6]     = {+1, -1, +1, +1, +1, +1};
int16_t OFFSET[6]  = { 90,  90,  90,  90,  90,  45};
int16_t MIN_ANG[6] = {  5,   5,  10,   5,   5,   0};
int16_t MAX_ANG[6] = {175, 175, 170, 175, 175, 100};
Servo    servo[6];

int16_t dhToServoDeg(int joint, float dh_deg){
  long v = (long)DIR[joint]* (long)dh_deg + (long)OFFSET[joint];
  if(v < MIN_ANG[joint]) v = MIN_ANG[joint];
  if(v > MAX_ANG[joint]) v = MAX_ANG[joint];
  return (int16_t)v;
}

void moveToPose(float q_deg[6], uint16_t ms_per_step=20, uint16_t steps=50){
  int16_t s[6], t[6];
  for(int i=0;i<6;++i){ s[i]=servo[i].read(); t[i]=dhToServoDeg(i,q_deg[i]); }
  for(uint16_t k=1;k<=steps;++k){
    float u=(float)k/steps; float e=u*u*(3-2*u); // smoothstep
    for(int i=0;i<6;++i){ int val = (int)(s[i] + e*(t[i]-s[i])); servo[i].write(val); }
    delay(ms_per_step);
  }
}

void setup(){
  for(int i=0;i<6;++i) servo[i].attach(PIN[i]);
  Serial.begin(115200);
  float q_home[6]={0,0,0,0,0,0};
  moveToPose(q_home,15,60);
  Serial.println(F("Send q in degrees as: q=a,b,c,d,e,f"));
  Serial.println(F("Example: q=30,35,-25,20,15,-40"));
}

void loop(){
  if(Serial.available()){
    String line=Serial.readStringUntil('\n'); line.trim();
    if(line.startsWith("q=")){
      float q[6]; int last=2;
      for(int i=0;i<6;++i){ int comma=line.indexOf(',',last);
        String tok=(comma==-1)? line.substring(last): line.substring(last,comma);
        q[i]=tok.toFloat(); last=comma+1; if(comma==-1) break; }
      moveToPose(q,15,60); Serial.println(F("OK"));
    }
  }
}
