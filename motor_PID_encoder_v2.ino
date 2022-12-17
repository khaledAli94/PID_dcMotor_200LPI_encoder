#include <Encoder.h>
#include <math.h>

#define IN1 11 // motor should rotate cw
#define IN2 10 // motor should rotate ccw
#define pwmpin 9
#define lower_limit 0
#define upper_limit 255
///////////////////////////
bool motor_state , target_is_reached;
int dir; 
///////////////////////////////////
unsigned long prevT; 
long eprev;  long currT;  long e ;
 float u ;  float pwr ;   float deltaT ;   float dedt ;  float eintegral;
/////////////////////////////////////
Encoder myEnc(2, 3); // if rotated cw acording to motor you would see an positive increment. its active High while no rotation
///////////////////////////////////////////////
// PID parameters
//float kp = 0.1425, ki = 0.345 , kd = 0.00335;
float kp = 0.125, ki = 0.065 , kd = 0.020;
long Pos_Buffer ;
long target = 1400;
////////////////////////////////////////////////
void readEncoder(){
  long newPosition = myEnc.read();
  Pos_Buffer= newPosition/4;
}
/////////////////////////////////////////////

void compute(){
 
   currT = micros();

  // time difference
   deltaT = ( (float)(currT - prevT)  ) / (1.0e6);
  prevT = currT;

  // error
   e = Pos_Buffer - target;

  // derivative
   dedt = (e - eprev) / (deltaT);

  // integral
   eintegral = eintegral +(e * deltaT);

  // control signal
    u = kp * e + kd * dedt + ki * eintegral;
  // motor power
   pwr = fabs(u);
  if (pwr > upper_limit){ pwr = upper_limit; }

  // motor direction
   dir = 1;
  if (u < 0){ dir = -1;}
  if (e == 0){turn_off();   target_is_reached = true;}
  else{ turn_on();          target_is_reached = false; }

  // signal the motor
  setMotor(dir, pwr);

  // store previous error
  eprev = e;
}
void turn_off(){
  motor_state = 0;
   analogWrite(pwmpin, 0);
   digitalWrite(IN1, 0) ; analogWrite(pwmpin, 0);
   digitalWrite(IN2, 0) ; analogWrite(pwmpin, 0);
}
void turn_on(){
  motor_state = 1;
}

void setMotor(int dir, float pwmVal){
  pwmVal = constrain(pwmVal, lower_limit, upper_limit);
     if(pwmVal>0 && pwmVal <20){ TCCR1B &= TCCR1B & B11111000 | B00000101; }  // for PWM frequency of 30.64 Hz
else if (pwmVal>20 && pwmVal <75){TCCR1B = TCCR1B & B11111000 | B00000100;  } // for PWM frequency of 122.55 Hz
else if (pwmVal> 75 && pwmVal <128) {TCCR1B = TCCR1B & B11111000 | B00000011;}  // for PWM frequency of 490.20 Hz (The DEFAULT)
else if (pwmVal>128 && pwmVal<200) {TCCR1B = TCCR1B & B11111000 | B00000001; } // set timer 1 divisor to 1 for PWM frequency of 31k37255 Hz
else if (pwmVal>200 && pwmVal<255){TCCR1B = TCCR1B & B11111000 | B00000010;  } // for PWM frequency of 3k921.16 Hz


    analogWrite(pwmpin, pwmVal);
  if (dir == 1 && motor_state == 1) {
     digitalWrite(IN1, 1) ; analogWrite(IN1, pwmVal);
     digitalWrite(IN2, 0) ; analogWrite(IN2, 0);
  }
  else if (dir == -1 && motor_state == 1){
    digitalWrite(IN1, 0) ; analogWrite(IN1, 0);
     digitalWrite(IN2, 1) ; analogWrite(IN2, pwmVal);
  }
  else{
  digitalWrite(IN1, 0) ; analogWrite(IN1, 0);
    digitalWrite(IN1, 0) ; analogWrite(IN2, 0);
  }
}


/////////////////////////
void setup() {
  Serial.begin(115200);
  pinMode(pwmpin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  readEncoder();
  compute();
  if (target==Pos_Buffer) {
  turn_off();
  }


        Serial.print("Time=");Serial.print(currT);Serial.print(","); Serial.print("\t");   
        Serial.print("pos=");Serial.print(Pos_Buffer); Serial.print(","); Serial.print("\t");   
        Serial.print("Error=");Serial.print(e); Serial.print(","); Serial.print("\t");   
        Serial.print("TDiff=");Serial.print(deltaT);Serial.print(",");  Serial.print("\t");   
        Serial.print("eDt=");Serial.print(dedt);Serial.print(",");  Serial.print("\t");   
        Serial.print("int=");Serial.print(eintegral);Serial.print(",");  Serial.print("\t");   
        Serial.print("ctrl=");Serial.print(u); Serial.print(",");    Serial.print("\t");   
        Serial.print("dir=");Serial.print(dir); Serial.print(",");    Serial.print("\t");   
        Serial.print("duty=");Serial.print(pwr); Serial.print("\t\n"); 

}
