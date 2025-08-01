// code for use with  StepperOnilne DM320T controller

int inPin = A0;
int supplyPin = 13;
int dirPin = 9;
int pulPin = 8;
// minimum direction change delay is 8 microseconds for DMT320T
int dirDelay = 10;
// minimum pulse length is 8 microseconds for DM320T
int pulDelay = 10;
long rpsMicro = 0;

boolean dirLow = true;

void setup() {
  pinMode(supplyPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pulPin, OUTPUT);
  pinMode(inPin, INPUT);

  digitalWrite(supplyPin,HIGH);
  digitalWrite(dirPin, LOW);

  Serial.begin(115200);
  Serial.println("<laserLab.ino>\n");
  
  float rpm = 33.0;
  float rps = rpm/60.0;

  //if rpsMicro > ~16000, havoc! provide for this.
  rpsMicro = (1000000.0/(rps * 400.0))/2.0 - pulDelay;
  delay(1000);

  Serial.println("Driving motor...");
  for(int i = 0; i < 200; i++){
    step();
  }

  delay(500);
  changeDir();

  Serial.println("Reversing...");
  for(int i = 0; i < 200; i++){
    step();
  }

  Serial.println("Input a number of steps to turn, negative to reverse:");
}

void changeDir(){
  dirLow = !dirLow;
  if(dirLow) digitalWrite(dirPin, LOW);
  else digitalWrite(dirPin, HIGH);
  delayMicroseconds(rpsMicro);
  delayMicroseconds(dirDelay);
}

void step(){
  digitalWrite(pulPin, LOW);
  delayMicroseconds(rpsMicro);
  delayMicroseconds(pulDelay);
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(rpsMicro);
  delayMicroseconds(pulDelay);
}

void loop() {
  if(Serial.available()){
    int stepInput = Serial.parseInt();
    long delayTime = Serial.parseInt();
    if(stepInput < 0) digitalWrite(dirPin, LOW);
    else digitalWrite(dirPin, HIGH);
    delayMicroseconds(dirDelay);
    stepInput = abs(stepInput);
    for(int i = 0; i < stepInput; i++){
      step();
      if (delayTime > 16383) {
        delay(delayTime / 1000);
      } else if (delayTime > 2) {
        delayMicroseconds(delayTime);
      }
      int data = analogRead(inPin);
      Serial.print("PD Analog read: ");
      // Serial.print(delayTime);
      // Serial.print(" ");
      Serial.println(data);
    }
    Serial.println("done");
    stepInput = 0;
  }
}


