const int statusPin = A5;
const int rotationPin = A4;
const int potentiometerPin = A3;

const int pwmPin = 9;
const int input1Pin = 1;
const int input2Pin = 2;

int statusValue = 0;
int rotationValue = 0;
int potentiometerValue = 0;
int lastStatusValue = 0;   
int motorEnabled = 0; 
int lastRotationDirection = 0;
int roatationEnable = 0;

void setup(){
  pinMode(statusPin,INPUT);
  pinMode(rotationPin,INPUT);
  pinMode(potentiometerPin,INPUT);

  pinMode(pwmPin, OUTPUT);
  pinMode(input1Pin, OUTPUT);
  pinMode(input2Pin,OUTPUT);
}

void loop(){
    statusValue = digitalRead(statusPin);
    rotationValue = digitalRead(rotationPin);
    potentiometerValue = analogRead(potentiometerPin);
    int motorSpeed = map(potentiometerValue, 0, 1023, 0, 255);

    if(rotationValue == HIGH && lastRotationDirection == LOW){
        roatationEnable = !roatationEnable;
    }
    lastRotationDirection = rotationValue;

    if (statusValue == HIGH && lastStatusValue == LOW) {
        motorEnabled = !motorEnabled;
    }
    lastStatusValue = statusValue;


    if(motorEnabled){
        if(roatationEnable){
            digitalWrite(input1Pin,HIGH);
            digitalWrite(input2Pin,LOW);
        }else{
            digitalWrite(input1Pin,LOW);
            digitalWrite(input2Pin,HIGH);
        };
        analogWrite(pwmPin, motorSpeed);
    }else {
        analogWrite(pwmPin, 0);
  }
   delay(50);
} 
