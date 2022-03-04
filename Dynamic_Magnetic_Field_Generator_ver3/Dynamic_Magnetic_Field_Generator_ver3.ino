// 20/12/07 ver 2.10 Kirihara
// 20/12/16 ver 2.20 Kirihara
// 20/12/23 ver 2.30 Kirihara
// 21/03/02 ver 2.35 Kirihara PID制御 CalcImput()内Kp Ki Kd を20穴プレート用に小さくしました．負担を下げます．
//                            Kiが入っているので止まってもしばらくすれば電圧上がって動くと思いますが気配がない場合連絡ください．
//                            電圧の変化のオーバーシュートが大きい場合はKdを減らしてください．途中で止まる場合はKiを上げてください．
//                            10穴プレートならKp=0.02f　Ki=0.1f Kd=0.02f 位がちょうどいいです．
// 21/10/** ver 3.00 Kirihara 時間変化関数を変更
#include "DualTB9051FTGMotorShield.h"

DualTB9051FTGMotorShield md;
const int LED = 4;
const float RPM_switch[4] = { 0.0f,30.0f,60.0f,240.0f }; //20穴の場合 30rpmで5Hz 60rpmで10Hz 240rpmで40Hz
const float timer_limit = 86400.0f; //86400秒=1日で停止．現在入力は「ImputV*ImputMulti()+ImputAdd()」　ImputMultiの中身は1-0.5*H((現在時刻)-timer_limit), ImputAddの中身は0にしてあります．
const float GeerRatio = 20.4f;

float timer_start2, timer_delta2;

//===============時間管理用変数 ProgramedRPM(millis())=================
const int mode = 0; //0でスイッチコントローラ式　1であらかじめプログラムした時間変化で進む
const float time_schedule[4] = { 1.0/60.0f,2.0f/60.0f,0.5/60.0f,0.5/60.0f }; //回転数を変更させる時間(開始からの累計時間ではなく，各区間の長さ:h)
const float RPM_schedule[4] = { 30.0f,-1.0f,60.0f,240.0f }; //回転数, time_scheduleと対応，-1.0で線形変化
int length=(int)sizeof(RPM_schedule) / sizeof(float);
float time_length;

//===============スイッチ読み取り変数 ReadSwitch()======================
//スイッチ読み取りピン;
float targ_rpm, targ_rpm_bf = 0; //変数初期化;
int n = 0; //同じく;
const int analogPin = 3; //3番ピンで読み取り;

//==============モーターパルス読み取り Countencorder()==================
//Green=encGND Blue=encVcc

//Yellow=encOUT1
const int outputA = 3;
//White=encOUT2
const int outputB = 5;
//エンコーダー信号
int aState, aLastState;

//================モーターパルス読み取り CalcRPM()=======================
//一回転パルス数
const float OneRotate = 12.0f;
//時間計測用関数
float timer_start1, timer_delta1;
//エンコーダーカウント
int counter = 0;

//================PID制御 CalcImput()=================================
//入力信号
float ImputV, ImputV_bf = 0;

//PIDの変数，値が大きいほど変化量が大きくなる
const float Kp = 0.02f;
const float Ki = 0.1f;
const float Kd = 0.02f;
float ImputVmax = 400;
float DeltaRpmBf = 0, DeltaRpmAf = 0;
float Integral;


//回転記録用
float rpm = 0, TempRpm = 0;

//ロータリースイッチを読み込んで目標rpmを返す;
void setup()
{
    Serial.begin(9600);
    Serial.print("\n");
    if (mode == 1) {
        if ((int)sizeof(RPM_schedule) / sizeof(float) != (int)sizeof(time_schedule) / sizeof(float)) {
            Serial.println("設定の時間と回転数の長さが一致しません");
        }
        Serial.print(Display_schedule());

        for (int i = 0;i < length;i++) {
            time_length += time_schedule[i];
        }
    }
    
    //Serial.println("test finish");

    //モータードライバ準備
    md.init();
    md.enableDrivers();
    delay(1);

    //エンコーダー準備
    pinMode(outputA, INPUT);
    pinMode(outputB, INPUT);
    aLastState = digitalRead(outputA);
    timer_start1 = millis();
    timer_start2 = millis();

    //スイッチ準備
    targ_rpm = 0;
}

void loop()
{
    //md.enableDrivers();
    //delay(1); // wait for drivers to be enabled so fault pins are no longer low
    md.setM1Speed(ImputV);
    Countencorder();
    timer_delta1 = millis() - timer_start1;
    timer_delta2 = millis() - timer_start2;
    //0.3秒ごとにモーター更新
    if (timer_delta1 > 300) {
        rpm = CalcRPM();
        if (mode == 0) {
            targ_rpm = ReadSwitch() * ImputMulti(millis()) + ImputAdd(millis() / 1000.0f);
        }
        else if (mode == 1) {
            targ_rpm = Programed_rpm(millis());
        }
        timer_start1 = millis();
    }
    if (timer_delta2 > 1000) {
        //データ表示 時間[s], 目標回転数[rpm],現在の回転数[rpm],入力電圧[V]で構成　1sごとに更新
        Serial.print(millis() / 1000.0f);
        Serial.print(",");
        //Serial.print("");
        Serial.print(targ_rpm);
        Serial.print(",");
        //Serial.print(": ");
        Serial.print(rpm);
        Serial.print(",");
        ImputV = CalcPID();
        //Serial.print("ImputV:");
        if (targ_rpm == 0 && rpm == 0) {
            ImputV = 0;
        }
        Serial.println(ImputV / 400 * 12);
        timer_start2 = millis();
    }
}

float ReadSwitch() {
    // put your main code here, to run repeatedly:
    float max_val = 1024.0f;
    float val = analogRead(analogPin);
    //Serial.print( " : " );
    if (val < max_val * 0.25f) {
        if (targ_rpm != RPM_switch[0]) {
            //Serial.print("Off\n");
        }
        return RPM_switch[0];
    }
    else if (max_val * 0.25f <= val && val < max_val * 0.5f) {
        if (targ_rpm != RPM_switch[1]) {
            //Serial.print("Low\n");
        }
        return RPM_switch[1];
    }
    else if (max_val * 0.5f <= val && val < max_val * 0.75f) {
        if (targ_rpm != RPM_switch[2]) {
            //Serial.print("Middle\n");
        }
        return RPM_switch[2];
    }
    else {
        if (targ_rpm != RPM_switch[3]) {
            //Serial.print("High\n");
        }
        return RPM_switch[3];
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
    timer_delta1 = millis() - timer_start1;
    float TempRpm = 60.0f / timer_delta1 * 1000.0f * counter / (OneRotate * GeerRatio);
    //計測値の初期化
    counter = 0;
    return TempRpm;
}

float CalcPID() {
    float p, i, d;
    DeltaRpmBf = DeltaRpmAf;
    DeltaRpmAf = targ_rpm - rpm;

    p = Kp * DeltaRpmAf;
    d = Kd * (DeltaRpmAf - DeltaRpmBf) / timer_delta1 * 1000;
    Integral += (DeltaRpmAf + DeltaRpmBf) / 2 * timer_delta1 / 1000;

    i = Ki * Integral;

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

float ImputMulti(float t) {
    if (t < timer_limit) {
        return 1;
    }
    else {
        return 0;
    }
}

float ImputAdd(float t) {
    return 0;
}

float Programed_rpm(float t) {
    float time = t / (3600000); //経過時間[h]の導出 60sec*60min*1000msで[ms]→[h]に変換
    float keika = 0;
    int phase = length;
    for (size_t i = 0; i < length; i++)
    {
        keika = keika + time_schedule[i];
        if (keika - time > 0) {
            phase = i;
            break;
        }
    }
    if (phase == 0) {
        if ((int)RPM_schedule[phase]==-1) {
            return RPM_schedule[0] / time_schedule[0] * time;
        }
        else
        {
            return RPM_schedule[phase];
        }
    }
    else if (phase == length-1) {
        if ((int)RPM_schedule[phase] == -1) {
            return -RPM_schedule[phase - 1] / time_schedule[phase] * (time-keika+time_schedule[phase]) + RPM_schedule[phase - 1];
        }
        else {
            return RPM_schedule[phase];
        }
    }
    else if (phase == length) {
        return 0.0f;
    }
    else {
        if ((int)RPM_schedule[phase] == -1) {
            return (RPM_schedule[phase + 1] - RPM_schedule[phase - 1]) / time_schedule[phase] * (time - keika + time_schedule[phase]) + RPM_schedule[phase - 1];
        }
        else {
            return RPM_schedule[phase];
        }
    }
}

String Display_schedule() {
    String message = "Time Schedule\n";
    for (int i = 0; i < sizeof(RPM_schedule) / sizeof(float); i++) {
        if (RPM_schedule[i] < 0 && RPM_schedule[i] > -1.5) {
            if (i == 0) {
                message += "0rpm~" + String(RPM_schedule[i]) + "rpm(" + String(time_schedule[i]) + "h)->";
            }
            else if (i == sizeof(RPM_schedule) / sizeof(float) - 1) {
                message += String(RPM_schedule[i]) + "rpm~0rpm" + "(" + String(time_schedule[i]) + "h)\n";
            }
            else {
                message += String(RPM_schedule[i - 1]) + "rpm~" + String(RPM_schedule[i + 1]) + "rpm" + "(" + String(time_schedule[i]) + "h)->";
            }
        }
        else if (i == sizeof(RPM_schedule) / sizeof(float) - 1) {
            message += String(RPM_schedule[i]) + "rpm" + "(" + String(time_schedule[i]) + "h)\n";
        }
        else {
            message += String(RPM_schedule[i]) + "rpm" + "(" + String(time_schedule[i]) + "h)->";
        }
    }
    return message;
}
