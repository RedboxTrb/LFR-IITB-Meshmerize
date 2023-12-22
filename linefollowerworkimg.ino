//final code for lfr
//speed of turn delay of turn
// reading delay in followline

//27
//Code snipped of workign analog lfr
//Array codes
//Theshold finder
int pins[7] = { A0, A1, A2, A3, A4, A5, A6 };
int sensorArray[7];
int sensorValue = 0;
double totalSum = 0, weightedSum = 0;
double distFromCenter[7] = { 0, -30, -20, -10, 10, 20, 30 };
int threshold = 170;
int maxi[7] = { 974, 965, 842, 975, 776, 968, 974  };
int mini[7] = { 183, 131, 116, 187, 115, 164, 178 };
int L1 = 2;
int L2 = 6;
int R1 = 4;
int R2 = 7;
int enA = 3;  //left
int enB = 5;  //right

int LED = 8;
int algoSwitch = 10;
int finalSwitch = 9;

int algoFlag = 0; //0 for lsrb, 1 for rslb
int finalFlag = 0;

int max_speed1 = 190;  //right enb
int max_speed2 = 160;  //left ena

//pid
double Kp = 15;  // 15
double Kd = 75;  // 25
double Ki = 0.001;
double P = 0;
double I = 0;
double D = 0;
double lastError = 0;
double lastI = 0;
double PID = 0;
int error;

char path[100];
int path_length;
int node = 0;

void readArray() {
  for (int i = 0; i < 7; i++) {
    sensorArray[i] = constrain(map(analogRead(pins[i]), mini[i], maxi[i], 0, 255), 0, 255);
  }

  sensorValue = 0;
  totalSum = 0;
  weightedSum = 0;
  for (int i = 0; i <= 6; i++) {
    totalSum += sensorArray[i];
    weightedSum += distFromCenter[i] * sensorArray[i];
    if (sensorArray[i] > threshold) sensorValue |= 1 << (6 - i);
  }
  //Serial.print("sensor value: ");
  //Serial.print(sensorValue);
  error = weightedSum / totalSum;
}
void calculatePID() {
  P = error;
  I = I + error;
  D = error - lastError;
  PID = P * Kp + I * Ki + D * Kd;
  lastError = error;
  // Set wheel movement based on errorValue
}

void pathSimplify()
{

  int B=1;

  while(B)
  {
    B = 0;
    int temp = 0; //variable to iterate through path
    int k=0;  //index to store values
    int minus=0; //to store size decrement in path_length
    for (temp; temp < path_length-2; temp++)
    {
      if (path[temp + 1] != 'B'){
        path[k++] = path[temp];
        continue;
      }
      int total = 0;
      int m;
      for (m=0; m <= 2; m++){
        switch(path[temp+m]){
          case 'R':
            total += 1;
            break;
          case 'S':
            break;
          case 'L':
            total += 3;
            break;
          case 'B':
            total += 2;
            break;
        }
      }
      total = total % 4;
      switch(total){
        case 0:
          path[k++] = 'S';
          break;
        case 1:
          path[k++] = 'R';
          break;
        case 2:
          path[k++] = 'B';
          B=1; //setting flag to run simplification again
          break;
        case 3:
          path[k++] = 'L';
          break;
      }
      minus += 2;
      temp+=2;
    }
    path[k++] = path[temp++];
    path[k++] = path[temp++];
    path_length -= minus;

    Serial.print("\nsimplification iteration: ");
    for (int c=0; c<path_length; c++)
    {
      Serial.print(path[c]);
      Serial.print(" ");
    }
    Serial.println();
  }

}

void stop(int d) {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(R1, 0);
  digitalWrite(R2, 0);
  digitalWrite(L1, 0);
  digitalWrite(L2, 0);
  delay(d);
}

void rollback(int d) {
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(R1, 0);
  digitalWrite(R2, 1);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  delay(d);
}
void followline() {
  error = 0;
  lastError = 0;

  while (1) {
    readArray();

    calculatePID();

    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    analogWrite(enB, constrain(max_speed1 - PID, 0, 255));
    analogWrite(enA, constrain(max_speed2 + PID, 0, 255));

    if (sensorValue == 0b0000000 || sensorValue == 0b1111111) {
      // rollback(50);
      delay(10);
      readArray();
      return;
    } if (sensorValue == 0b1111100 || sensorValue == 0b1111110 || sensorValue == 0b1001111 || sensorValue == 0b1011111 || sensorValue == 0b1000111 || sensorValue == 0b1111000 || sensorValue == 0b0111111) {
      //  rollback(50);
      delay(10);
      readArray();
      return;
    } if (sensorValue == 0b0111100 || sensorValue == 0b0111110 || sensorValue == 0b0001111 || sensorValue == 0b0011111 || sensorValue == 0b0000111 || sensorValue == 0b0111000) {
      //  rollback(50);
      delay(10);
      readArray();

      return;
    }
  }
  // delay(4);
}
void leftTurn(int d) {
  readArray();
  analogWrite(enA, 140);
  analogWrite(enB, 110);
  while (sensorArray[2] < threshold) {
    digitalWrite(R1, 1);
    digitalWrite(R2, 0);
    digitalWrite(L1, 0);
    digitalWrite(L2, 1);
    readArray();
    delay(d);
  }
  //fro small return
  digitalWrite(R1, 0);
  digitalWrite(R2, 1);
  digitalWrite(L1, 1);
  digitalWrite(L2, 0);
  delay(50);
}

void rightTurn(int d) {
  readArray();
  analogWrite(enA, 180);
  analogWrite(enB, 180);
  while (sensorArray[5] < threshold) {
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    readArray();
    delay(d);
  }
  //for small return
  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  delay(50);
}
void UTurn(int d) {
  // do {
  analogWrite(enA, 180);
  analogWrite(enB, 210);
  while (sensorArray[5] < threshold) {
    digitalWrite(R1, 0);
    digitalWrite(R2, 1);
    digitalWrite(L1, 1);
    digitalWrite(L2, 0);
    delay(d);

    readArray();
  }
  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 0);
  digitalWrite(L2, 1);
  delay(50);
}
void moveforward(int d) {
  analogWrite(enA, 100);
  analogWrite(enB, 130);

  digitalWrite(R1, 1);
  digitalWrite(R2, 0);
  digitalWrite(L1, 1);
  digitalWrite(L2, 0);
  delay(d);
}
void rightfirst() {

  // Stop on all white or all black

  // Serial.println("right first called");
  int sensor1 = sensorArray[1];
  int sensor5 = sensorArray[5], sensor6 = sensorArray[6], sensor4 = sensorArray[4];
  int prevValue = sensorValue;
  // moveforward(50);
  readArray();
  if(sensorValue==0b1111110 || sensorValue==0b1111111 ||sensorValue==0b1111100  )
  {
    end();
  }

  else if ((sensor6 > threshold || sensor5 > threshold) && sensor4 > threshold) {

    // Serial.println("Right turn");
    rightTurn(10);
    path[path_length++] = 'R';
  }

  else if (sensor6 > threshold) {

    // Serial.println("Right turn");
    rightTurn(10);
    path[path_length++] = 'R';
  }

  else if (sensorArray[3] > threshold || sensorArray[4] > threshold) {
    // Serial.println("Straight");
    path[path_length++] = 'S';
  }

  else if (sensor1 > threshold) {
    // Serial.println("Left turn");
    leftTurn(10);
    path[path_length++] = 'L';
  }

  else if (sensorValue == 0b0000000) {
    // Serial.println("U turn");
    UTurn(10);
    path[path_length++] = 'B';
  }
}

void leftfirst() {

  // Stop on all white or all black

  // Serial.println("right first called");
  int sensor1 = sensorArray[1], sensor2 = sensorArray[2], sensor3 = sensorArray[3];
  int sensor5 = sensorArray[5], sensor6 = sensorArray[6], sensor4 = sensorArray[4];
  // moveforward(50);
  readArray();
  if(sensorValue==0b1111110 || sensorValue==0b1111111 ||sensorValue==0b1111100  )
  {
    end();
  }

  else if ((sensor1 > threshold || sensor2 > threshold) && sensor3 > threshold) {

    // Serial.println("Left turn");
    leftTurn(10);
    path[path_length++] = 'L';
  }

  else if (sensor1 > threshold) {
    // Serial.println("Left turn");
    leftTurn(10);
    path[path_length++] = 'L';
  }

  else if (sensorArray[3] > threshold || sensorArray[4]>threshold) {
    // Serial.println("Straight");
    path[path_length++] = 'S';
  }

  else if (sensor6 > threshold) {

    // Serial.println("Right turn");
    rightTurn(10);
    path[path_length++] = 'R';
  }

  else if (sensorValue == 0b0000000) {
    // Serial.println("U turn");
    UTurn(10);
    path[path_length++] = 'B';
  }
}

void finalrun(){
  readArray();
  delay(10);
  if (sensorValue != 0 && sensorValue != 0b111111) {
    followline();
    //Serial.println("followline");
  }
  // Serial.print("sensorValue");
  // Serial.println(sensorValue, BIN);

  if (node == path_length)
    end2();

  switch(path[node++]){
    case 'R':
      stop(100);
      rightTurn(10);
      break;
    case 'L':
      stop(100);
      leftTurn(10);
      break;
    case 'S':
      break;
  }
}

void end()
{
  // Serial.print("\npath: ");
  // int c;
  // for (c=0; c<path_length; c++)
  // {
  //   Serial.print(path[c]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("\nafter simplification: ");
  // for (c=0; c<path_length; c++)
  // {
  //   Serial.print(path[c]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  pathSimplify();
  digitalWrite(LED, HIGH);

  while(!digitalRead(finalSwitch))
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    delay(100);
  }
  digitalWrite(LED, LOW);
  finalFlag = 1;
  return;
}

void end2()
{
  digitalWrite(LED, HIGH);
  while(1)
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    delay(100);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(algoSwitch, INPUT_PULLUP);
  pinMode(finalSwitch, INPUT_PULLUP);

  if(digitalRead(algoSwitch))
    algoFlag = 1;

}
void loop() {

  readArray();
  delay(10);
  if (sensorValue != 0 && sensorValue != 0b111111) {
    followline();
    //Serial.println("followline");
  }
  stop(100);


  // Serial.print("sensorValue");
  // Serial.println(sensorValue, BIN);
  if (finalFlag)
  {
    if (node == path_length)
      end2();

    switch(path[node++]){
      case 'R':
        rightTurn(10);
        break;
      case 'L':
        leftTurn(10);
        break;
      case 'S':
        break;
    }
  }
  else{
    if (algoFlag)
      rightfirst();
    else
      leftfirst();

  }
}
