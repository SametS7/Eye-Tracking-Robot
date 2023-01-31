#include <WiFi.h>
#include "Arduino.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>


const char* ssid           = "ESP32-CTDP";                // SSID Name
const char* password       = "Commandor62";   // SSID Password - Set to NULL to have an open AP
const int   channel        = 10;                        // WiFi Channel number between 1 and 13
const bool  hide_SSID      = false;                     // To disable SSID broadcast -> SSID will not appear in a basic WiFi scan
const int   max_connection = 3;  

volatile int posR = 0;
volatile int posL = 0;
long prevT = 0;
float eprevR = 0;
float eprevL = 0;
float eintegralR = 0;
float eintegralL = 0;
float pi=3.14159;
int pwr_R=250;  //
int pwr_L=250;
int targetL=0;
int targetR=0;
long duration;
float distanceCm;

int t_s = 0;
int b_s = 0;
int r_s = 0;
int l_s = 0;
int x = 0;
int lock = 0;


#define SOUND_SPEED 0.034
#define trigPin  15
#define echoPin  16
#define MOTOR_R_PIN_1   25  //pin number
#define MOTOR_R_PIN_2   26    //pin number

#define ENCA_R 13 // YELLOW
#define ENCB_R 14 // WHITE
#define PWM_R  21


#define MOTOR_L_PIN_1 32    //pin number
#define MOTOR_L_PIN_2 33    //pin number

#define ENCA_L 18 // YELLOW
#define ENCB_L 19 // WHITE
#define PWM_L  27


WiFiUDP Udp; 
unsigned int Port = 4210;
char incomingPacket[200];

unsigned long previousMillis = 0;
const long interval = 600;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);



//Mesafe Sensörü
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input






//Sağ motor

  pinMode(ENCA_R,INPUT);
  pinMode(ENCB_R,INPUT);
 attachInterrupt(digitalPinToInterrupt(ENCA_R),readEncoderRight,RISING); 

  pinMode(PWM_R, OUTPUT);
  pinMode(MOTOR_R_PIN_1,OUTPUT);
  pinMode(MOTOR_R_PIN_2,OUTPUT);

//Sol Motor

  pinMode(ENCA_L,INPUT);
  pinMode(ENCB_L,INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA_L),readEncoderLeft,RISING);

  pinMode(PWM_L, OUTPUT);
  pinMode(MOTOR_L_PIN_1,OUTPUT);
  pinMode(MOTOR_L_PIN_2,OUTPUT);


  // Set up the AP mode and configure the AP's parameters
  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, channel, hide_SSID, max_connection);
    Serial.println(WiFi.softAPIP());
 
 Udp.begin(Port);
}


void setMotorRight(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if (distanceCm <= 5){
    digitalWrite(in1,LOW);    //durma
    digitalWrite(in2,LOW);
  }
  else{
    if(dir == 1){               //ileri
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    }
    else if(dir == -1){         //geri
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    }
    else{
      digitalWrite(in1,LOW);    //durma
      digitalWrite(in2,LOW);
    }
  }  
}

void setMotorLeft(int dir, int pwmVal, int pwm, int in3, int in4){
  analogWrite(pwm,pwmVal);
  if(distanceCm <= 5){
    digitalWrite(in3,LOW);    //durma
    digitalWrite(in4,LOW);
  }  
  else{
  
    if(dir == 1){               //ileri
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    }
    else if(dir == -1){         //geri
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
    }
  
    else{
      digitalWrite(in3,LOW);    //durma
      digitalWrite(in4,LOW);
    }  

  }
}

ICACHE_RAM_ATTR void readEncoderRight(){
  int br = digitalRead(ENCB_R);
  if(br > 0){
    posR++;
  }
  else{
    posR--;
  }
}


ICACHE_RAM_ATTR void readEncoderLeft(){
  int bl = digitalRead(ENCB_L);
  if(bl > 0){
    posL++;
  }
  else{
    posL--;
  }
}


void loop() {

unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);


    }

    String data = incomingPacket;

      
    if(x == 8){
      lock++;
      x = 0;
    }
    if(data == "top"){
      if (x == 0 || x == 4){
        x++;
      }
      t_s++;
     
      if(t_s >=5){
         r_s = 0;
      l_s = 0;
      b_s = 0;
        targetL = posL;
        targetR = posR;
        targetL = targetL + 99999;
        targetR = targetR + 99999;
      }
      if(t_s >= 8){
        targetL = posL;
        targetR = posR;
        t_s = 0;
      }
    }
    else if(data == "bot"){
      if(x == 2 || x == 6){
        x++;
      }
     
      b_s++;
      if(b_s >= 5){
        t_s = 0;
        r_s = 0;
        l_s = 0;
        targetL = posL;
        targetR = posR;
        targetL = targetL - 99999;
        targetR = targetR - 99999;
      }
      if(b_s >= 8){
        targetL = posL;
        targetR = posR;
        b_s = 0;
      }
    }
    else if(data == "right"){
      if(x == 3 || x == 7){
        x++;
      }

      r_s++;
      if((r_s%6) >=5){
        t_s = 0;
        b_s = 0;
        l_s = 0;
        targetL = posL;
        targetR = posR;
        targetL = targetL + 72;
        targetR = targetR - 72;
      }
    }
    else if(data == "left") {
      if(x == 1 || x == 5){
        x++;
      }
      
      l_s++;
      if((l_s%6)>=5){
        t_s = 0;
        b_s = 0;
        r_s = 0;
        targetL = posL;
        targetR = posR;
        targetR = targetR + 72;
        targetL = targetL - 72;
      }
    }

    else if(data == "mid"){
      x = 0;
      r_s = 0;
      l_s = 0;
    }

    if((lock%2)== 1){
      targetL = posL;
      targetR = posR;
      pwr_L = 0;
      pwr_R = 0;
    }
    else{
      pwr_R = 50;
      pwr_L = 50;  
    }


    previousMillis = currentMillis;
  }



// Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);






  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

    // Read the position
  int pos = 0; 
  int pos1 = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posL;
  pos1 =posR; 
  interrupts(); // turn interrupts back on
  // error
  
  int eL = pos - targetL;
  // derivative
  float dedtL = (eL-eprevL)/(deltaT);
   // integral
  eintegralL = eintegralL + eL*deltaT;
  // control signal
  float uL = kp*eL + kd*dedtL + ki*eintegralL;



  int eR = pos1 - targetR;
  // derivative
  float dedtR = (eR-eprevR)/(deltaT);
  // integral
  eintegralR = eintegralR + eR*deltaT;
  // control signal
  float uR = kp*eR + kd*dedtR + ki*eintegralR;


  // motor direction L
  int dirL = 1;
  if(uL<100 and uL >(-100)){
    dirL = 0;
   }
  else if (uL>(100)) {
    dirL = -1;
    }


  // motor direction R
  int dirR = 1;
  if(uR<100 and uR>(-100)){
    dirR = 0;
   }
  else if (uR>(100)) {
    dirR= -1;
    }


  // signal the motor
  setMotorRight(dirR,pwr_R,PWM_R,MOTOR_R_PIN_1,MOTOR_R_PIN_2);
  setMotorLeft(dirL,pwr_L,PWM_L,MOTOR_L_PIN_1,MOTOR_L_PIN_2);

  // store previous error
  eprevL = eL;
  eprevR = eR;
  Serial.print(targetL);
  Serial.println(" ");
  Serial.print(targetR);
  Serial.println();
  Serial.print(posL);
  Serial.println(" ");
  Serial.print(posR);
  Serial.println();
Serial.print(pwr_L) ;
  Serial.println();
Serial.print( pwr_R) ;
  Serial.println();
}
