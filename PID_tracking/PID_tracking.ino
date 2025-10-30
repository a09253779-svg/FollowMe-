#include <Servo.h>
#define maxsize 10
Servo myservo;

const int trigL=2;
const int echoL=3;
const int trigR=4;
const int echoR=5;
const int servoPin=9;

float Kp=4.0;
float Ki=0.2;
float Kd=1.0;

int minPos=0;
int maxPos=180;
float barrier=2.0; 
int smoothingN=7;
float bufL[maxsize],bufR[maxsize];
int idxL=0,idxR=0;

float integral=0.0;
float prevError=0.0;
unsigned long prevTime=0;

float servoPosF=90.0; 
int servoPos=90;

void setup(){
  myservo.attach(servoPin);
  myservo.write(servoPos);

  pinMode(trigL, OUTPUT);pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT);pinMode(echoR, INPUT);

  for(int i=0;i<10;i++) 
    bufL[i]=bufR[i]=50.0; 
  prevTime=millis();
}

float readDistance(int trigPin,int echoPin){
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  long duration=pulseIn(echoPin,HIGH,30000); 
  if(duration==0) 
    return -1.0;
  float distanceCm=(duration/2.0)*0.0343;
  return distanceCm;
}

float smooth(float *buf,int &idx,float val,int N){
  buf[idx]=val;
  idx=(idx+1)%N;
  float s=0;
  int count=0;
  for(int i=0;i<N;i++) {
    if(buf[i]>0) {
      s+=buf[i];
      count++;
    }
  }
  return count>0?s/count:-1.0;
}

void loop(){
  float rawL = readDistance(trigL, echoL);
  delay(30);
  float rawR = readDistance(trigR, echoR);
  delay(10);

  if(rawL<0) 
    rawL=bufL[(idxL+(smoothingN-1))%smoothingN];
  if(rawR<0) 
    rawR=bufR[(idxR+(smoothingN-1))%smoothingN];

  float dL=smooth(bufL,idxL,rawL,smoothingN);
  float dR=smooth(bufR,idxR,rawR,smoothingN);
  float error = dL - dR;


  if(abs(error)<barrier){
    integral*=0.9;
  } 
  else{
    unsigned long now=millis();
    float dt=(now-prevTime)/1000.0;
    if(dt<=0) 
      dt=0.001;
    prevTime=now;

    integral+=error*dt;
    float integralMax=50.0;
    if(integral>integralMax) 
      integral=integralMax;
    if(integral<-integralMax) 
      integral=-integralMax;

    float derivative=(error-prevError)/dt;
    prevError=error;

    float output=Kp*error+Ki*integral+Kd*derivative;
    float maxStep=5.0;
    if(output>maxStep) 
      output=maxStep;
    if(output<-maxStep) 
      output=-maxStep;

    servoPosF=constrain(servoPosF+output, (float)minPos,(float)maxPos);
    servoPos=int(round(servoPosF));
    myservo.write(servoPos);
  }

  delay(50); 
}