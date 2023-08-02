#include <SharpIR.h>

//infrarouge pin
#define IRPinLeft A3
#define IRPinRight A2

#define left 0
#define right !left
//ultraSon pin
#define echoPin 6
#define trigPin 7

#define boutonPin 2//bouton pin

#define minDis 20 //la distance minimum ultrason
#define minDisInfrarouge 20//la distance minimum Infrarouge
#define maxDis 150 //la distance maximum ultrason
#define sec 1200  //delai
#define vSon 59 //valeur de temps en µs d'un aller retour du son sur 1cm

//********//

bool running_ = true;
int vitesse =60;
int turningDelay =sec*1.8;
int turningSpeed =vitesse;

//

SharpIR sensorLeft( SharpIR::GP2Y0A02YK0F, IRPinLeft );
SharpIR sensorRight( SharpIR::GP2Y0A02YK0F, IRPinRight );

//structure 
struct Motor{
  int brake,direction_,speed_;
  } ;
//declaration des deux motors
struct Motor motor2,motor1;


void setup() {
  //les pins des motors
  motor1.brake = 9;
  motor1.direction_= 12;
  motor1.speed_ = 3;
  motor2.brake=8;
  motor2.direction_=13;
  motor2.speed_ =11;
  Serial.begin(9600);
  //initialiser les pins des motors
  pinMode(motor1.brake, OUTPUT); //Initiates Motor Channel A pin
  pinMode(motor2.brake, OUTPUT); //Initiates Motor Channel A pin
  pinMode(motor1.direction_, OUTPUT); //Initiates Brake Channel A pin
  pinMode(motor2.direction_, OUTPUT); //Initiates Brake Channel A pin
  pinMode(motor1.speed_, OUTPUT); //Initiates Brake Channel A pin
  pinMode(motor2.speed_, OUTPUT); //Initiates Brake Channel A pin
  //initailiser pin boutton
  pinMode(boutonPin,INPUT_PULLUP);
  //initailiser pin ultrason  
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
}

void loop() {
  int distanceFace;
  bool b = digitalRead(boutonPin);
  if(!b){
    running_ = !running_; 
    if(!running_){
      arreter();
      delay(1000);
    }
   }
   if (!running_){
    Serial.println("Not running");
    return;
  }
  //calculer la distance entre l ultrason et l'obstacle
  distanceFace = getSoundDis();
  Serial.print("distanceFace:");
  Serial.println(distanceFace);

  //si le robot est plus loin de l'obstacle ,le robot avance
  if(distanceFace < 0 || distanceFace > minDis){
    avancer(vitesse);
  }
  //si le robot est proche de l'obstacle on calcule la distance par les deux infrarouges pour choisir la direction de rotation
  else{
    int disRight = sensorRight.getDistance();
    int disLeft = sensorLeft.getDistance();
    //delay(100);
    Serial.print("disLeft:");
    Serial.println(disLeft);
    
    Serial.print("disRight:");
    Serial.println(disRight);

    //si les deux distances calculer par l'infrarouge inférieur à minDisInfrarouge le robot marche en arriére
    if(disLeft <= minDisInfrarouge && disRight<=minDisInfrarouge){
      moveBack(vitesse,disRight,disLeft);
    }
    //choisir la direction de rotation
    else{
      arreter(); 
      if(disLeft < disRight){
        Serial.println("turning right");
        tourner(turningDelay,right,turningSpeed);
        arreter();
      }
      else{
        Serial.println("turning left");
        tourner(turningDelay,left,turningSpeed);
        arreter();
      }
    }
  }
}

void avancer(int vitesse){
  digitalWrite(motor1.brake,LOW);
  digitalWrite(motor2.brake,LOW);
  digitalWrite(motor1.direction_,HIGH);
  digitalWrite(motor2.direction_,HIGH);
  digitalWrite(motor1.speed_,vitesse);
  digitalWrite(motor2.speed_,vitesse);
}

int getSoundDis(){
  int sum = 0;
  int iters = 5;
  for(int i=0;i<iters;i++){
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
   
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    int dur = pulseIn(echoPin,HIGH);
    delay(50);
    sum+=dur;
  }
  sum /= iters;
  return sum/vSon;  //vSon :valeur de temps pour parcourir 1cm en microseconde
}

void arreter(){
  digitalWrite(motor1.brake,HIGH);
  digitalWrite(motor2.brake,HIGH);
}
void moveBack(int vitesse,int disRight,int disLeft){
  while(disRight <minDisInfrarouge && disLeft<minDisInfrarouge){
      digitalWrite(motor1.brake,LOW);
      digitalWrite(motor2.brake,LOW);
      digitalWrite(motor1.direction_,LOW);
      digitalWrite(motor2.direction_,LOW);
      digitalWrite(motor1.speed_,vitesse);
      digitalWrite(motor2.speed_,vitesse);
      disRight=sensorRight.getDistance();
      disLeft = sensorLeft.getDistance();
  }
  if(disRight<disLeft){
      tourner(turningDelay,left,turningSpeed);
      arreter();
  }
  else{
    tourner(turningDelay,right,turningSpeed);
    arreter();
  }
}

void tourner(int duration,int dir,int vitesse){
  digitalWrite(motor1.brake,LOW);
  digitalWrite(motor2.brake,LOW);
  digitalWrite(motor1.direction_,dir?HIGH:LOW);
  digitalWrite(motor2.direction_,!dir?HIGH:LOW);
  digitalWrite(motor1.speed_,vitesse);
  digitalWrite(motor2.speed_,vitesse);
  delay(duration);
}
