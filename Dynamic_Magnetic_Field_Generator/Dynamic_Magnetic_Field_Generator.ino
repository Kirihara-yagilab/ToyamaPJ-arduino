// 20/12/07 ver 2.10 Kirihara
// 20/12/16 ver 2.20 Kirihara

#include "DualTB9051FTGMotorShield.h"

DualTB9051FTGMotorShield md;
int LED = 4;
float SetRPM[4]={0.0f,30.0f,60.0f,240.0f}; //ギア比20.4の場合
float timer_limit=3600.0f; //3600秒で停止．現在入力は「ImputV*ImputMulti()+ImputAdd()」　ImputMultiの中身は1-H_{1/2}((現在時刻)-timer_limit), ImputAddの中身は0にしてあります．
float GeerRatio = 20.4f;

//float SetRPM[4] = {0.0f, 37.5f * 2, 75.0f * 2, 300.0f * 2};
//===============スイッチ読み取り変数 ReadSwitch()======================
//スイッチ読み取りピン
int analogPin = 3;
float targ_rpm, targ_rpm_bf = 0;
int n = 0;
//==============モーターパルス読み取り Countencorder()==================

// Green=encGND Blue=encVcc
//Yellow=encOUT1
#define outputA 3
//White=encOUT2
#define outputB 5
//エンコーダー信号
int aState, aLastState;

//================モーターパルス読み取り CalcRPM()=======================
//一回転パルス数
float OneRotate = 12.0f;
//時間計測用関数
float timer_start, timer_delta;
//エンコーダーカウント
int counter = 0;


//================PID制御 CalcImput()=================================
//入力信号
float ImputV, ImputV_bf = 0;
#define Kp 0.02f
#define Ki 0.1f
#define Kd 0.02f
float ImputVmax = 400;
float DeltaRpmBf = 0, DeltaRpmAf = 0;
float Integral;


//回転記録用
float rpm = 0, TempRpm = 0;

//ロータリースイッチを読み込んで目標rpmを返す;
void setup()
{
  Serial.begin(9600);
    while (!Serial) {
        digitalWrite(13, LED);
        delay(200);
        LED = 1 - LED;
    }
  
  //モータードライバ準備
  //Serial.println("Dual TB9051FTG Motor Shield");
  md.init();
  md.enableDrivers();
  delay(1); // wait for drivers to be enabled so fault pins are no longer low
  // Uncomment to flip a motor's direction:
  //md.flipM1(true);
  //md.flipM2(true);

  //エンコーダー準備
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  aLastState = digitalRead(outputA);
  timer_start = millis();

  //スイッチ準備
  targ_rpm = 0;
}

void loop()
{
  //md.enableDrivers();
  //delay(1); // wait for drivers to be enabled so fault pins are no longer low
  md.setM1Speed(ImputV);
  Countencorder();
  timer_delta = millis() - timer_start;
  //0.3秒ごとにモーター更新
  if (timer_delta > 300) {
    rpm = CalcRPM();
    targ_rpm = ReadSwitch();
    //Serial.print("現在の時間rpm: ");
    Serial.print(millis() / 1000.0f);
    Serial.print(",");
    //Serial.print("目標rpm:");
    Serial.print(targ_rpm);
    Serial.print(",");
    //Serial.print("現在の回転数rpm: ");
    Serial.print(rpm);
    Serial.print(",");
    ImputV = CalcPID();
    ImputV=ImputV*ImputMulti(millis())+ImputAdd(millis() / 1000.0f);
    //Serial.print("ImputV:");
    if (targ_rpm == 0 && rpm == 0) {
      ImputV = 0;
    }
    Serial.println(ImputV / 400 * 12);
  }
  //md.disableDrivers();
  //delay(1);
}

float ReadSwitch() {
  // put your main code here, to run repeatedly:
  float max_val = 1024.0f;
  float val = analogRead(analogPin);
  //Serial.print( " : " );
  if (val < max_val * 0.25f) {
    if (targ_rpm != SetRPM[0]) {
      //Serial.print("Off\n");
    }
    return SetRPM[0];
  }
  else if (max_val * 0.25f <= val && val < max_val * 0.5f) {
    if (targ_rpm != SetRPM[1]) {
      //Serial.print("Low\n");
    }
    return SetRPM[1];
  }
  else if (max_val * 0.5f <= val && val < max_val * 0.75f) {
    if (targ_rpm != SetRPM[2]) {
      //Serial.print("Middle\n");
    }
    return SetRPM[2];
  }
  else {
    if (targ_rpm != SetRPM[3]) {
      //Serial.print("High\n");
    }
    return SetRPM[3];
  }
}

void Countencorder() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState && aState != 0) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
    }
    else {
      counter++;
    }
    //Serial.print("Position: ");
    //Serial.println(counter);
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
}

void stopIfFault() {
  if (md.getM1Fault()) {
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault()) {
    Serial.println("M2 fault");
    while (1);
  }
}

float CalcRPM() {
  timer_delta = millis() - timer_start;
  float TempRpm = 60.0f / timer_delta * 1000.0f * counter / (OneRotate * GeerRatio);
  //計測値の初期化
  counter = 0;
  timer_start = millis();

  return TempRpm;
}

float CalcPID() {
  float p, i, d;
  DeltaRpmBf = DeltaRpmAf;
  DeltaRpmAf = targ_rpm - rpm;

  p = Kp * DeltaRpmAf;
  d = Kd * (DeltaRpmAf - DeltaRpmBf) / timer_delta * 1000;
  Integral += (DeltaRpmAf + DeltaRpmBf) / 2 * timer_delta / 1000;

  i = Ki * Integral;

  /*
    if(targ_rpm==0){
    if(rpm==0){
      return 0;
    }
    }
  */
  if (p + i + d > 0) {
    if (p + i + d < ImputVmax) {
      return p + i + d;
    }
    else {
      return ImputVmax;
    }
  }
  else {
    return 0;
  }
}

float ImputMulti(float t){
  if(t<timer_limit){
    return 1;
  }
  else{
    return 0;
  }
}

float ImputAdd(float t){
  return 0;
}
