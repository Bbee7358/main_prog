#include <Arduino.h>
#include <timer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

//モーター1  右前
#define PWM1 11 //ピン11を「PWM4」とする
#define INB1 25 //ピン25を「INB4」とする
#define INA1 27 //ピン27を「INA4」とする
//モーター2 左前
#define PWM2 3  //ピン3を「PWM1」とする
#define INB2 9  //ピン9を「INB1」とする
#define INA2 13 //ピン13を「INA1」とする

//モーター3 左後ろ
#define PWM3 5  //ピン5を「PWM2」とする
#define INB3 15 //ピン15を「INB2」とする
#define INA3 17 //ピン17を「INB2」とする

//モーター4 右後ろ
#define PWM4 7  //ピン7を「PWM3」とする
#define INB4 19 //ピン19を「INB3」とする
#define INA4 23 //ピン23を「INA3」とする

//スタートボタン
#define SWICH 52 //ピン47を「SWICH」とする

//前ボールセンサー
#define FBall A0

//右ボールセンサー
#define RBall A1

//後ろボールセンサー
#define BBall A2

//左ボールセンサー
#define LBall A3

/*LINE1(ブロック番号)3(列番号。。。大きい方が外側)*/
//ライン1（前）
#define LINE13 A3
#define LINE12 A1
//ライン2（右）
#define LINE23 A2
#define LINE22 A4
#define LINE21 A6
//ライン3（後ろ）
#define LINE33 A9
#define LINE32 A7
#define LINE31 A5
//ライン4（左）
#define LINE43 A12
#define LINE42 A10
#define LINE41 A8

//LED
#define LEDdesu 8

//ライン処理
int L_x[2]; //ラインが反応したときにそのラインの座標を記録する。一回目を[0]、二回目を[1]とする([]には0か1が入る)
int L_y[2]; //ラインが反応したときにそのラインの座標を記録する。一回目を[0]、二回目を[1]とする([]には0か1が入る)
int Lnum;   //ラインに反応した回数
int Lpast = 0;  //前に反応したラインの番号を記録する

int a = 0;   // aはint型初期値は0（ステートのボックス）
int aa = 0;  // aaは0（サブファイルの変わり）
int b = 999; // bはint型初期値は999（初期化のための変数）
int c = 0;   // cはint型初期値は0（ボールセンサの番号）
int d = 0;   // dはint型初期値は0（スタートスイッチのダブルタッチ防止の変数
int e = 0;   //eはボールを見て動くのか、ラインを見て動くのかを決めるためのフラグ　（e = 0の時はボールを見て判断するとき、1の時はラインを見て判断するとき）
int f = 1;   //ロボットが回り込みをしていたかを記録し、回り込みの長さを決める変数
int g = 1;   //前回までロボットがしていたことを記録する（ラインセンサーが反応したときにどっち方向に行けばいいかを決めるため）
int h = 999; //タイマーでステートを動かす時間を決める時にステートを一時的に決めhに入る。そして、時間が経ったらaにｈを代入する
int i = 0;   //もし動作しているステートにいるときにラインが反応したらそれ以上違うステートに行くのをやめるために0から1にする
int j = 0;   //ラインから離れる時、直前に左右どちらに行っていたかを記録する変数(i = 0の時は、右センサが命令していたとき。i = 1の時は、左センサが命令していたとき。)
int m = 0;   //最終、モーターの出力を変換した場合に、負の数に戻すためのフラグ

unsigned long LineResetT = 0; // lineの時間をはかり、2秒経ったらjをリセット（999）にする

int ir[17];          //ボールの値
uint8_t ir_8bit[17]; //ボールのデジタルがレジスタで入る
int ir_digital[17];  //ボールのデジタル値の逆の値が入る
int L = 1000;        //ボールの値を数える回数
float BAngle = 0;    //ボールの角度
int BValue = 0;      // eはint型初期値は0（ボールセンサ値）

//モーターで回り込みの時にボールの方へ向かわす要素を入れる変数
int BAtack1;
int BAtack2;
int BAtack3;
int BAtack4;

//モーター比
float MotorR1 = 1;   //右前
float MotorR2 = 1;   //左前
float MotorR3 = 0.8; //左後ろ
float MotorR4 = 0.8; //右後ろ

//モーター出力
int Motor1;
int Motor2;
int Motor3;
int Motor4;

//モーターパワー
float MPOWER6 = 1.6;
float MPOWER5 = 1.5;
float MPOWER4 = 1.4;
float MPOWER3 = 1.2;

//ラインの境目（コートによってこの値をいじる）
int LineValue = 300;

// Dirの値を記録
int DIR0;   // 0°
int DIR20;  // 20°
int DIR90;  // 90°
int DIR180; // 180°
int DIR270; // 270°
int DIR340; // 340°

float GoDir; //ロボットの進行方向を入れる

long LINEMtime = 0;       //"KINEMtimeL""KINEMtimeS"のどちらかの値が入り、最終的に何秒間ラインから離れるかを決める
int KINEMtimeL = 200;     //ラインから離れるときに何秒間離れるか（longVer）
int KINEMtimeS = 100;     //ラインから離れるときに何秒間離れるか（shortVer）
unsigned long DoTime = 0; //ロボットを動かす時間

// PD制御
float NewDeviationDir = 0; //偏差値（最新の）
float OldDeviationDir = 0; //偏差値（一個前の偏差値）
int TargetDir;             //目標値（角度）
int P;                     // P制御の最終値
float D;                   // D制御の最終値
float Kp = 0.5;            //比例ゲイン(動作併用用)
float Kd = 0;              //微分ゲイン(動作併用用)
float kkp = 1.5;           //比例ゲイン(姿勢制御だけ用)
float kkd = 134;           //微分ゲイン(姿勢制御だけ用)
unsigned long NowTime;     // D制御で回転時間を出すために
unsigned long OldTime;     // D制御で回転時間を出すために

int hata = 0;  //プログロムの状態を見たいときに自由に使う（プログラムには必要ない）

int serial = 1; //シリアルモニターを表示させるときはここを0から1にする

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  //モーター1 右前
  pinMode(PWM1, OUTPUT); // PWM
  pinMode(INB1, OUTPUT); // INB
  pinMode(INA1, OUTPUT); // INA

  //モーター2 左前
  pinMode(PWM2, OUTPUT); // PWM
  pinMode(INB2, OUTPUT); // INB
  pinMode(INA2, OUTPUT); // INA

  //モーター3 左後ろ
  pinMode(PWM3, OUTPUT); // PWM
  pinMode(INB3, OUTPUT); // INB
  pinMode(INA3, OUTPUT); // INA

  //モーター4 右後ろ
  pinMode(PWM4, OUTPUT); // PWM
  pinMode(INB4, OUTPUT); // INB
  pinMode(INA4, OUTPUT); // INA

  //スタートボタン
  pinMode(SWICH, INPUT); //スタートボタンを入力に設定

  //前ボールセンサー
  pinMode(FBall, INPUT); //前ボールセンサーを入力に設定

  //右ボールセンサー
  pinMode(RBall, INPUT); //右ボールセンサーを入力に設定

  //後ろボールセンサー
  pinMode(BBall, INPUT); //後ろボールセンサーを入力に設定

  //左ボールセンサー
  pinMode(LBall, INPUT); //左ボールセンサーを入力に設定

  //ライン1（前）
  pinMode(LINE13, INPUT);
  pinMode(LINE12, INPUT);

  //ライン2（左）
  pinMode(LINE23, INPUT);
  pinMode(LINE22, INPUT);
  pinMode(LINE21, INPUT);

  //ライン3（後ろ）
  pinMode(LINE33, INPUT);
  pinMode(LINE32, INPUT);
  pinMode(LINE31, INPUT);

  //ライン4（右）
  pinMode(LINE43, INPUT);
  pinMode(LINE42, INPUT);
  pinMode(LINE41, INPUT);

  pinMode(LEDdesu,OUTPUT);
};

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event; // eventという定義

  bno.getEvent(&event); //方位センサー(BNO055)の値を検出する

  if (aa == 0)
  {             //スタートするまでのステート <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    digitalWrite(LEDdesu,HIGH);
    if (a == 0) //ステート0 キャリブレーション スイッチが押されてない時に押されるまで待つ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        b = a;
      };

      analogWrite(PWM1, Motor1); //右前モータ
      analogWrite(PWM2, Motor2); //左前モータ
      analogWrite(PWM3, Motor3); //左後ろモータ
      analogWrite(PWM4, Motor4); //右後ろモータ

      //ストップ
      digitalWrite(INA1, LOW);
      digitalWrite(INB1, LOW); //停止

      digitalWrite(INA2, LOW);
      digitalWrite(INB2, LOW); //停止

      digitalWrite(INA3, LOW);
      digitalWrite(INB3, LOW); //停止

      digitalWrite(INA4, LOW);
      digitalWrite(INB4, LOW); //停止

      DIR0 = event.orientation.x; //方位センサー(BNO055)の正面方向の値をSTRAIGHT_DIRECTIONに設定 (キャリブレーション）
      TargetDir = DIR0;           //目標角度を真正面方向に設定

      if (digitalRead(SWICH) == LOW)
      {             //スイッチが押されたら
        delay(100); //チャタリング防止のため
        a = 1;      //チャタリングしていないかの確認
      };
    }
    else if (a == 1) //ボタンが指で離されたのを確認したら2回目のスイッチが押されるのを待つステートに行く
    {
      if (a != b) //初期化
      {
        b = a;
      };

      if (digitalRead(SWICH) == LOW)
      {        // 0.1秒経ってもスイッチが押されていたらチャタリングではないと判断
        a = 4; //スイッチが離されるのを待つ
      }
      else
      {
        a = 0; //チャタリングしていたため、リセット
      };
    }
    else if (a == 4)
    {             //二回目のスイッチを押されるのを待つために一回目の指を離したのを検知する
      if (a != b) //初期化
      {
        b = a;
      };

      if (digitalRead(SWICH) == HIGH)
      { //指がスイッチから離れたら
        delay(100);
        a = 5;
      }
    }
    else if (a == 5)
    {
      if (a != b) //初期化
      {
        b = a;
      };

      if (digitalRead(SWICH) == HIGH)
      { //チャタリングしていなかったら
        a = 6;
      }
      else
      {
        a = 0;
      }
    }
    else if (a == 6)
    {             // 2回目のスイッチ待ち
      if (a != b) //初期化
      {
        b = a;
      };

      if (digitalRead(SWICH) == LOW)
      {             //スイッチが押されたら
        delay(100); //チャタリング防止のため
        a = 7;      //チャタリングしていないかの確認
      };
    }
    else if (a == 7)
    {             //チャタリングの確認
      if (a != b) //初期化
      {
        b = a;
      };

      // Dirの設定
      DIR20 = DIR0 + 20; // 20゜
      if (360 < DIR20)
      { //角度が360を超えている数値になってしまったら
        DIR20 = DIR20 - 360;
      };
      DIR90 = DIR0 + 90; // 90゜
      if (360 < DIR90)
      { //角度が360を超えている数値になってしまったら
        DIR90 = DIR90 - 360;
      };
      DIR180 = DIR0 + 180; // 180°
      if (360 < DIR180)
      { //角度が360を超えている数値になってしまったら
        DIR180 = DIR180 - 360;
      };
      DIR270 = DIR0 + 270; // 270゜
      if (360 < DIR270)
      { //角度が360を超えている数値になってしまったら
        DIR270 = DIR270 - 360;
      };
      DIR340 = DIR0 + 340; // 340°
      if (360 < DIR340)
      { //角度が360を超えている数値になってしまったら
        DIR340 = DIR340 - 360;
      };

      TargetDir = DIR0; //目標角度を真正面方向に設定

      if (digitalRead(SWICH) == LOW)
      {          // 0.1秒経ってもスイッチが押されていたらチャタリングではないと判断
        aa = 10; // mainプログラムスタート！
      }
      else
      {
        a = 6; //チャタリングしていたため、リセット
      };
    }
  }
  else if (aa == 10) //ラインの検出、方位の確認、ボールの値を検出、どこのステートに行くかを決める <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  {
    digitalWrite(LEDdesu,LOW);
    //ロボットを速く動かす（ボールを追う、ラインから離れる）時のモーターの設定
    if (analogRead(LINE13) > LineValue) //前にラインがある
    {
      if (Lpast != 13) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        hata = 0;
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 0;
          L_y[Lnum] = 3;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 0;
          L_y[Lnum] = 3;
          Lnum += 1;  //回数を数える
          Lpast = 13; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE12) > LineValue) //前にラインがある
    {
      if (Lpast != 12) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 0;
          L_y[Lnum] = 2;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 0;
          L_y[Lnum] = 2;
          Lnum += 1;  //回数を数える
          Lpast = 12; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE23) > LineValue) //左にラインがある
    {
      if (g == 0) //ボールを追いかける動作を前回していたら
      {
        if (Lpast != 23) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
        {
          if (Lnum >= 1) // 2回目のラインの反応があったら行動する
          {
            //2回目のラインの座標を記録
            L_x[Lnum] = -3;
            L_y[Lnum] = 0;

            //ラインの反応からロボットの動いた方向を分析
            if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
            {
              if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
              {
              GoDir = 0; //上側に進むようにする
              }
              else //上側にロボットが進んでいたら
              {
              GoDir = 180; //下側に進むようにする
              }
            }
            else
            {
              if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
              {
                if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
                {
                  GoDir = 90; //右側に進むようにする
                }
                else //右側にロボットが進んでいたら
                {
                  GoDir = -90; //左側に進むようにする
                }
              }
              else
              {
                if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
                {
                  GoDir = 45; //右斜め前進
                }
                else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
                {
                  GoDir = 135; //右斜め後進
                }
                else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
                {
                  GoDir = -45; //左斜め前進
                }
                else //右斜め前に動いていたら
                {
                  GoDir = -135; //左斜め後進
                };
              };
            };

            e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
            Lnum = 0;  //ラインの回数をリセット
            i = 0; //実行ステートに移動できるようにする
          }
          else
          {
            //ラインセンサの座標を設定
            L_x[Lnum] = -3;
            L_y[Lnum] = 0;
            Lnum += 1;  //回数を数える
            Lpast = 23; // 2回目以降にこのセンサを見ないようにするため

            i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
          }

          LINEMtime = KINEMtimeL;

          LineResetT = millis();
          LineResetT += 2000; //ラインがあったらタイマースタート
        }
      }
    }
    else if (analogRead(LINE22) > LineValue) //左にラインがある
    {
      if (Lpast != 22) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = -2;
          L_y[Lnum] = 0;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = -2;
          L_y[Lnum] = 0;
          Lnum += 1;  //回数を数える
          Lpast = 22; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE21) > LineValue) //左にラインがある
    {
      if (Lpast != 21) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = -1;
          L_y[Lnum] = 0;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
             if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = -1;
          L_y[Lnum] = 0;
          Lnum += 1;  //回数を数える
          Lpast = 21; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE33) > LineValue) //後ろにラインがある
    {
      if (Lpast != 33) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 33) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 0;
          L_y[Lnum] = -3;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 0;
          L_y[Lnum] = -3;
          Lnum += 1;  //回数を数える
          Lpast = 33; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE32) > LineValue) //後ろにラインがある
    {
      if (Lpast != 32) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 0;
          L_y[Lnum] = -2;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 0;
          L_y[Lnum] = -2;
          Lnum += 1;  //回数を数える
          Lpast = 32; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE31) > LineValue) //後ろにラインがある
    {
      if (Lpast != 31) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 0;
          L_y[Lnum] = -1;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 0;
          L_y[Lnum] = -1;
          Lnum += 1;  //回数を数える
          Lpast = 31; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE43) > LineValue) //右にラインがある
    {
      if (Lpast != 43) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 3;
          L_y[Lnum] = 0;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 3;
          L_y[Lnum] = 0;
          Lnum += 1;  //回数を数える
          Lpast = 43; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE42) > LineValue) //右にラインがある
    {
      if (Lpast != 42) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 2;
          L_y[Lnum] = 0;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 2;
          L_y[Lnum] = 0;
          Lnum += 1;  //回数を数える
          Lpast = 42; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else if (analogRead(LINE41) > LineValue) //右にラインがある
    {
      if (Lpast != 41) //前回反応したセンサーがこのセンサーでなければ（同じセンサーでラインを認識してしまうとロボットの動きが見れないから）
      {
        if (Lnum >= 1) // 2回目のラインの反応があったら行動する
        {
          //2回目のラインの座標を記録
          L_x[Lnum] = 1;
          L_y[Lnum] = 0;

          //ラインの反応からロボットの動いた方向を分析
          if (L_x[0] == 0 && L_x[1] == 0) // x軸には変化がなかったら
          {
            if (L_y[0] > L_y[1]) //下側にロボットが進んでいたら
            {
              GoDir = 0; //上側に進むようにする
            }
            else //上側にロボットが進んでいたら
            {
              GoDir = 180; //下側に進むようにする
            }
          }
          else
          {
            if (L_y[0] == 0 && L_y[1] == 0) // y軸には変化がなかったら
            {
              if (L_x[0] > L_x[1]) //左側にロボットが進んでいたら
              {
                GoDir = 90; //右側に進むようにする
              }
              else //右側にロボットが進んでいたら
              {
                GoDir = -90; //左側に進むようにする
              }
            }
            else
            {
              if (L_x[0] > L_x[1] && L_y[0] > L_y[1]) //左斜め後ろに動いていたら
              {
                GoDir = 45; //右斜め前進
              }
              else if (L_x[0] > L_x[1] && L_y[0] < L_y[1]) //左斜め前に動いていたら
              {
                GoDir = 135; //右斜め後進
              }
              else if (L_x[0] < L_x[1] && L_y[0] > L_y[1]) //右斜め後ろに動いていたら
              {
                GoDir = -45; //左斜め前進
              }
              else //右斜め前に動いていたら
              {
                GoDir = -135; //左斜め後進
              };
            };
          };

          e = 1;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
          Lnum = 0;  //ラインの回数をリセット
          i = 0; //実行ステートに移動できるようにする
        }
        else
        {
          //ラインセンサの座標を設定
          L_x[Lnum] = 1;
          L_y[Lnum] = 0;
          Lnum += 1;  //回数を数える
          Lpast = 41; // 2回目以降にこのセンサを見ないようにするため

          i = 1; //ラインセンサーが反応したので次の行動をラインから離れるという行動にしたいので、センサーをチェックするのをやめる
        }

        LINEMtime = KINEMtimeL;

        LineResetT = millis();
        LineResetT += 2000; //ラインがあったらタイマースタート
      }
    }
    else //ラインが反応していなかったら
    {
      hata = 1;
      if (i == 0) //ラインが動作するステートにいるときに反応していなかったら
      {
        if (DIR90 < DIR270) //-------------------------------------------------------------------------------------  問題
        {
          if (DIR90 < event.orientation.x && event.orientation.x < DIR270) // 180度から±90°の方向を向いていたら
          {
            if (DIR90 < event.orientation.x && event.orientation.x <= DIR180) //右側を向いていたら
            {
              h = 8; //左回転（方向修正）
            }
            else
            {
              h = 9; //右回転（方向修正）
            };
          }
          else // 0度から±90°の方向を向いていたら
          {
            // analog値リセット
            for (int m = 0; m <= 16; m++)
            {
              ir[m] = 0; //ボールの値をリセット　×16
            }

            for (int k = 0; k < L; k++)
            {
              ir_8bit[1] = PIND & _BV(7);  //前
              ir_8bit[2] = PING & _BV(1);  //右前前
              ir_8bit[3] = PINL & _BV(7);  //右前
              ir_8bit[4] = PINL & _BV(5);  //右前右
              ir_8bit[5] = PINL & _BV(3);  //右
              ir_8bit[6] = PINL & _BV(2);  //右後右
              ir_8bit[7] = PINL & _BV(4);  //右後
              ir_8bit[8] = PINL & _BV(6);  //右後後
              ir_8bit[9] = PING & _BV(0);  //後
              ir_8bit[10] = PING & _BV(2); //左後後
              ir_8bit[11] = PINC & _BV(0); //左後
              ir_8bit[12] = PINC & _BV(2); //左後左
              ir_8bit[13] = PINC & _BV(4); //左
              ir_8bit[14] = PINC & _BV(5); //左前左
              ir_8bit[15] = PINC & _BV(3); //左前
              ir_8bit[16] = PINC & _BV(1); //左前前

              for (int m = 0; m <= 16; m++)
              {
                if (ir_8bit[m] > 0) //ボールの値がHIGHだったら
                {
                  ir_digital[m] = 0; //なぜ1ではなく0にするかというと、ボールセンサがボールが近くにあるほど0が出るため、近づくとボールの値が高くなるようにするため
                }
                else //ボールの値がLOWだったら
                {
                  ir_digital[m] = 1;
                };
              };

              // irの値を格納する変数にIRのセンサー値を足していく
              for (int m = 0; m <= 16; m++)
              {
                ir[m] += ir_digital[m];
              };
            };

            if (ir[1] > ir[2])
            {
              BValue = ir[1];
              c = 1; //前
            }
            else
            {
              BValue = ir[2];
              c = 2;
            };
            for (int k = 2; k <= 16; k++)
            {
              if (ir[k] > BValue)
              {
                BValue = ir[k];
                c = k;
              };
            };

            if (c > 8)
            { //左側をむいていたら
              BAngle = 22.5 * (c - 1) - 360;
            }
            else
            { //右側を向いていたら
              BAngle = 22.5 * (c - 1);
            };

            if (BValue >= 400) //一番高いセンサーが、ある一定の反応をしていたら
            {
              e = 0;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
              if (16 <= c || c <= 2) //前のボールセンサがボールを見ていたら
              {
                if (-90 <= BAngle && BAngle < 0) //ボールが-90°~0°の方向にあったら→左側前進
                {
                  GoDir = BAngle; //進む方向を決める（今回はボールに向かって進みたいから進行方向はボールの方向）
                }
                else if (0 <= BAngle && BAngle < 90) //ボールが0°~90°の方向にあったら→右側前進
                {
                  GoDir = BAngle; //進む方向を決める（今回はボールに向かって進みたいから進行方向はボールの方向）
                }
              }
              else //ボールが回り込まなければいけないとこにあったら
              {
                if (BAngle > 0) //右側にボールがあったら
                {
                  GoDir = BAngle + 3 * BValue / 5 - 300;
                }
                else
                {
                  GoDir = BAngle - 3 * BValue / 5 + 300;
                }
                if (GoDir > 180) // 180°以降の数字は本来-となるべきなので左側仕様のマイナスにしなければならない
                {
                  GoDir = -360 + GoDir;
                }
                else if (GoDir < -180) //-180°以降の数字は本来+となるべきなので右側仕様のプラスにしなければならない
                {
                  GoDir = 360 + GoDir;
                }
              };

              if (0 <= GoDir && GoDir < 90) //ボールが-180°~-90°の方向にあったら
              {
                h = 10; //右側前進
              }
              else if (90 <= GoDir && GoDir < 180) //ボールが-90°~0°の方向にあったら
              {
                h = 20; //右側後進
              }
              else if (0 > GoDir && GoDir >= -90) //ボールが0°~90°の方向にあったら
              {
                h = 30; //左側前進
              }
              else //ボールが90°~180°の方向にあったら
              {
                h = 40; //左側後進
              };
            }
            else //周りにボールがなかったら
            {
              h = 7; //姿勢制御
            };
          };
        }
        else // DIR90の方がDIR270よりおおきかったら
        {
          if (DIR270 <= event.orientation.x && event.orientation.x <= DIR90) //前を向いていたら
          {
            // analog値リセット
            for (int m = 0; m <= 16; m++)
            {
              ir[m] = 0; //ボールの値をリセット　×16
            }

            for (int k = 0; k < L; k++)
            {
              ir_8bit[1] = PIND & _BV(7);  //前
              ir_8bit[2] = PING & _BV(1);  //右前前
              ir_8bit[3] = PINL & _BV(7);  //右前
              ir_8bit[4] = PINL & _BV(5);  //右前右
              ir_8bit[5] = PINL & _BV(3);  //右
              ir_8bit[6] = PINL & _BV(2);  //右後右
              ir_8bit[7] = PINL & _BV(4);  //右後
              ir_8bit[8] = PINL & _BV(6);  //右後後
              ir_8bit[9] = PING & _BV(0);  //後
              ir_8bit[10] = PING & _BV(2); //左後後
              ir_8bit[11] = PINC & _BV(0); //左後
              ir_8bit[12] = PINC & _BV(2); //左後左
              ir_8bit[13] = PINC & _BV(4); //左
              ir_8bit[14] = PINC & _BV(5); //左前左
              ir_8bit[15] = PINC & _BV(3); //左前
              ir_8bit[16] = PINC & _BV(1); //左前前

              for (int m = 0; m <= 16; m++)
              {
                if (ir_8bit[m] > 0) //ボールの値がHIGHだったら
                {
                  ir_digital[m] = 0; //なぜ1ではなく0にするかというと、ボールセンサがボールが近くにあるほど0が出るため、近づくとボールの値が高くなるようにするため
                }
                else //ボールの値がLOWだったら
                {
                  ir_digital[m] = 1;
                };
              };

              // irの値を格納する変数にIRのセンサー値を足していく
              for (int m = 0; m <= 16; m++)
              {
                ir[m] += ir_digital[m];
              };
            };

            if (ir[1] > ir[2])
            {
              BValue = ir[1];
              c = 1; //前
            }
            else
            {
              BValue = ir[2];
              c = 2;
            };
            for (int k = 2; k <= 16; k++)
            {
            if (ir[k] > BValue)
              {
                BValue = ir[k];
                c = k;
              };
            };

            if (c > 8)
            { //左側をむいていたら
              BAngle = 22.5 * (c - 1) - 360;
            }
            else
            { //右側を向いていたら
              BAngle = 22.5 * (c - 1);
            };

            if (BValue >= 400) //一番高いセンサーが、ある一定の反応をしていたら
            {
              e = 0;  //進む方向を決めるときにボールによって決められたのか、ラインによって決められたのかを判断するため
              if (16 <= c || c <= 2) //前のボールセンサがボールを見ていたら
              {
                if (-90 <= BAngle && BAngle < 0) //ボールが-90°~0°の方向にあったら→左側前進
                {
                  GoDir = BAngle; //進む方向を決める（今回はボールに向かって進みたいから進行方向はボールの方向）
                }
                else if (0 <= BAngle && BAngle < 90) //ボールが0°~90°の方向にあったら→右側前進
                {
                  GoDir = BAngle; //進む方向を決める（今回はボールに向かって進みたいから進行方向はボールの方向）
                }
              }
              else //ボールが回り込まなければいけないとこにあったら
              {
                if (BAngle > 0) //右側にボールがあったら
                {
                  GoDir = BAngle + 3 * BValue / 5 - 300;
                }
                else
                {
                  GoDir = BAngle - 3 * BValue / 5 + 300;
                }
                if (GoDir > 180) // 180°以降の数字は本来-となるべきなので左側仕様のマイナスにしなければならない
                {
                  GoDir = -360 + GoDir;
                }
                else if (GoDir < -180) //-180°以降の数字は本来+となるべきなので右側仕様のプラスにしなければならない
                {
                  GoDir = 360 + GoDir;
                }
              };

              if (0 <= GoDir && GoDir < 90) //ボールが-180°~-90°の方向にあったら
              {
                h = 10; //右側前進
              }
              else if (90 <= GoDir && GoDir < 180) //ボールが-90°~0°の方向にあったら
              {
                h = 20; //右側後進
              }
              else if (0 > GoDir && GoDir >= -90) //ボールが0°~90°の方向にあったら
              {
                h = 30; //左側前進
              }
              else //ボールが90°~180°の方向にあったら
              {
                h = 40; //左側後進
              };
            }
            else //周りにボールがなかったら
            {
              h = 7; //姿勢制御
            };
          }
          else //後ろを見ていたら
          {
            if (DIR180 < DIR90) // DIR90~DIR180の間に0°の境目がある時
            {
              if (DIR180 < event.orientation.x && event.orientation.x < DIR270) //左側を向いていたら
              {
                h = 9; //右回転（方向修正）
              }
              else //右側をむいていたら
              {
                h = 8; //左回転（方向修正）
              };
            }
            else // DIR180~DIR270の間に0°がある時
            {
              if (DIR90 < event.orientation.x && event.orientation.x < DIR180) //右側を向いていたら
              {
                h = 8; //左回転（方向修正）
              }
              else //左側を向いていたら
              {
                h = 9; //右回転（方向修正）
              };
            };
          };
        };
      };
    };

    if (i == 0) //ラインが反応してなかったら実行するが、ラインが反応するとある一定の回数ラインを読むまで実行できない
    {
      NowTime = millis(); //下の条件式でエラーが出たので今の時間を変数に代入してから条件を聞くことにした
      //    Serial.print("aa:"); Serial.print(aa);
      //    Serial.print(" a:"); Serial.print(a);
      //    Serial.print(" i:"); Serial.print(i);
      //    Serial.print(" NT:"); Serial.print(NowTime);
      //    Serial.print(" LT:"); Serial.print(LINEMtime);
      //    Serial.print(" DT:"); Serial.println(DoTime);
      if (DoTime < NowTime)
      {                                                    //一番最初にここに来るときはDoTimeは0だから条件は成立する
        OldDeviationDir = NewDeviationDir;                 //一個前の偏差値を比較
        NewDeviationDir = TargetDir - event.orientation.x; //偏差値 = 目標角度（真正面方向） - 今の角度
        if (NewDeviationDir < -180)
        { // 180~360
          NewDeviationDir = NewDeviationDir + 360;
        }
        else if (180 <= NewDeviationDir)
        { // 180°~360°
          NewDeviationDir = NewDeviationDir - 360;
        };

        P = NewDeviationDir; // P制御
        NowTime = millis();
        D = (OldDeviationDir - NewDeviationDir) / (NowTime - OldTime); // D制御
        OldTime = NowTime;                                             // D制御の時に回転時間を使いたいため

        if(e != 0)  //ボールによって方向が決められていたら
        {
          if (0 <= GoDir && GoDir < 90) //ボールが-180°~-90°の方向にあったら
          {
            h = 50; //右側前進
          }
          else if (90 <= GoDir && GoDir < 180) //ボールが-90°~0°の方向にあったら
          {
            h = 60; //右側後進
          }
          else if (0 > GoDir && GoDir >= -90) //ボールが0°~90°の方向にあったら
          {
            h = 70; //左側前進
          }
          else //ボールが90°~180°の方向にあったら
          {
            h = 80; //左側後進
          };
        }

        aa = 20; //動作をするステートブロック
        a = h;   //ステートを動かしている時間にステートの行先（ｈ）を決め、そこに行く
      };
    };
  }
  else if(aa == 12)  //ラインから離れるときうまく離れられているかの経過を見る
  {
    if (analogRead(LINE13) > LineValue) //前にラインがある
    {
      if(L_y[0] == 3)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE12) > LineValue) //前にラインがある
    {
      if(L_y[0] == 2)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE23) > LineValue) //左にラインがある
    {
      if(L_x[0] == -3)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE22) > LineValue) //左にラインがある
    {
      if(L_x[0] == -2)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE21) > LineValue) //左にラインがある
    {
      if(L_x[0] == -1)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE33) > LineValue) //後ろにラインがある
    {
      if(L_y[0] == -3)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE32) > LineValue) //後ろにラインがある
    {
      if(L_y[0] == -2)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE31) > LineValue) //後ろにラインがある
    {
      if(L_y[0] == -1)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE43) > LineValue) //右にラインがある
    {
      if(L_x[0] == 3)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE42) > LineValue) //右にラインがある
    {
      if(L_x[0] == 2)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }
    else if (analogRead(LINE41) > LineValue) //右にラインがある
    {
      if(L_x[0] == 1)  //離れる瞬間にみた、一番最初にみたラインがこのラインセンサーであったら
      {
        aa = 10;
      }
    }

    if (DoTime < NowTime)  //ようわからんけどラインから離れる動作をする時間が経ったら、もう強制的に全センサーを見るサブステートに行く
    {
      aa = 10;  //全センサー
    };
  }
  else if (aa == 15)
  { //最終的に動作をするところ
    // motorの向きを設定（あらかじめ前のステートでモーターに±をつけ、それから正転、逆転を判断）
    if (Motor1 > 0)
    {
      digitalWrite(INA1, HIGH);
      digitalWrite(INB1, LOW); //正転
      m = 0;                   //変換していないというフラグ
    }
    else
    {
      digitalWrite(INA1, LOW);
      digitalWrite(INB1, HIGH); //逆転
      Motor1 = Motor1 * (-1);   //モーター値を正の数に直す（モータ値だから正の数でないといけない）
      m = 1;                    //モーターの値を負の数から正の数に直したのでそれをもとに戻すためのフラグ
    };
    if (Motor1 == 0)
    { //もし停止しろ（0）と言われたら停止
      digitalWrite(INA1, LOW);
      digitalWrite(INB1, LOW); //停止
      m = 0;                   //変換していないというフラグ
    };
    if (Motor1 > 250)
    { //もし255を超えると、値が一からになってしまうので、ならないようにするプログラム
      Motor1 = 250;
    };
    analogWrite(PWM1, Motor1); //右前モータ
    if (m == 1)                //負の数から正の数に変換したので、負の数に戻す
    {
      Motor1 = Motor1 * -1;
    }

    if (Motor2 > 0)
    {
      digitalWrite(INA2, HIGH);
      digitalWrite(INB2, LOW); //正転
      m = 0;                   //変換していないというフラグ
    }
    else
    {
      digitalWrite(INA2, LOW);
      digitalWrite(INB2, HIGH); //逆転
      Motor2 = Motor2 * (-1);   //モーター値を正の数に直す（モータ値だから正の数でないといけない）
      m = 1;                    //モーターの値を負の数から正の数に直したのでそれをもとに戻すためのフラグ
    };
    if (Motor2 == 0)
    { //もし停止しろ（0）と言われたら停止
      digitalWrite(INA2, LOW);
      digitalWrite(INB2, LOW); //停止
      m = 0;                   //変換していないというフラグ
    };
    if (Motor2 > 250)
    { //もし255を超えると、値が一からになってしまうので、ならないようにするプログラム
      Motor2 = 250;
    };
    analogWrite(PWM2, Motor2); //右前モータ
    if (m == 1)                //負の数から正の数に変換したので、負の数に戻す
    {
      Motor2 = Motor2 * -1;
    }

    if (Motor3 > 0)
    {
      digitalWrite(INA3, HIGH);
      digitalWrite(INB3, LOW); //正転
      m = 0;                   //変換していないというフラグ
    }
    else
    {
      digitalWrite(INA3, LOW);
      digitalWrite(INB3, HIGH); //逆転
      Motor3 = Motor3 * (-1);   //モーター値を正の数に直す（モータ値だから正の数でないといけない）
      m = 1;                    //モーターの値を負の数から正の数に直したのでそれをもとに戻すためのフラグ
    };
    if (Motor3 == 0)
    { //もし停止しろ（0）と言われたら停止
      digitalWrite(INA3, LOW);
      digitalWrite(INB3, LOW); //停止
      m = 0;                   //変換していないというフラグ
    };
    if (Motor3 > 250)
    { //もし255を超えると、値が一からになってしまうので、ならないようにするプログラム
      Motor3 = 250;
    };
    analogWrite(PWM3, Motor3); //右前モータ
    if (m == 1)                //負の数から正の数に変換したので、負の数に戻す
    {
      Motor3 = Motor3 * -1;
    }

    if (Motor4 > 0)
    {
      digitalWrite(INA4, HIGH);
      digitalWrite(INB4, LOW); //正転
      m = 0;                   //変換していないというフラグ
    }
    else
    {
      digitalWrite(INA4, LOW);
      digitalWrite(INB4, HIGH); //逆転
      Motor4 = Motor4 * -1;   //モーター値を正の数に直す（モータ値だから正の数でないといけない）
      m = 1;                    //モーターの値を負の数から正の数に直したのでそれをもとに戻すためのフラグ
    };
    if (Motor4 == 0)
    { //もし停止しろ（0）と言われたら停止
      digitalWrite(INA4, LOW);
      digitalWrite(INB4, LOW); //停止
      m = 0;                   //変換していないというフラグ
    };
    if (Motor4 > 250)
    { //もし255を超えると、値が一からになってしまうので、ならないようにするプログラム
      Motor4 = 250;
    };
    analogWrite(PWM4, Motor4); //右前モータ
    if (m == 1)                //負の数から正の数に変換したので、負の数に戻す
    {
      Motor4 = Motor4 * -1;
    }

    if(e == 0)  //ボールによって進行方向が決められていたら
    {
      aa = 10;  //全センサーの動きを見る
    }
    else  //ラインによって進行方向が決められていたら
    {
      aa = 12;  //ラインから離れるときのラインの経過を見る
    }
  }
  else if (aa == 20)
  {             //動作をするステートブロック <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if (a == 7) //姿勢制御 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 1; //姿勢制御をしたことを記録する（ラインから離れる場合に必要になるため）
        NowTime = millis();
        DoTime = NowTime + 100; // 100秒
        b = a;
      };

      //モーター出力＝PD制御
      Motor1 = kkp * P - kkd * D;
      Motor2 = kkp * P - kkd * D;
      Motor3 = kkp * P - kkd * D;
      Motor4 = kkp * P - kkd * D;

      aa = 15; //動作を決める
    }
    else if (a == 8) //ステート8  （方向修正 左回転）~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 1; //姿勢制御をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      //モーター出力＝モーター比×モーターパワー
      Motor1 = -250;
      Motor2 = -250;
      Motor3 = -250;
      Motor4 = -250;

      aa = 15; //動作を決める
    }
    else if (a == 9) //ステート9 （方向修正 右回転）~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 1; //姿勢制御をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      //モーター出力＝モーター比×モーターパワー
      Motor1 = 250;
      Motor2 = 250;
      Motor3 = 250;
      Motor4 = 250;

      aa = 15; //動作を決める
    }
    else if (a == 10) // 0°~90°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = (20 / 9 * GoDir - 100) * MPOWER5 * MotorR1 + Kp * P - Kd * D;  //右前
      Motor2 = 100 * MPOWER5 * MotorR2 + Kp * P - Kd * D;                     //左前
      Motor3 = (-20 / 9 * GoDir + 100) * MPOWER5 * MotorR3 + Kp * P - Kd * D; //左後ろ
      Motor4 = -100 * MPOWER5 * MotorR4 + Kp * P - Kd * D;                    //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 20) // 90°~180°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100;

      Motor1 = 100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack1;                     //右前
      Motor2 = (-20 / 9 * GoDir + 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack2; //左前
      Motor3 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack3;                    //左後ろ
      Motor4 = (20 / 9 * GoDir - 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack4;  //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 30) // 0°~-90°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D;                    //右前
      Motor2 = (20 / 9 * GoDir + 100) * MPOWER5 * MotorR2 + Kp * P - Kd * D;  //左前
      Motor3 = 100 * MPOWER5 * MotorR3 + Kp * P - Kd * D;                     //左後ろ
      Motor4 = (-20 / 9 * GoDir - 100) * MPOWER5 * MotorR4 + Kp * P - Kd * D; //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 40) //-90°~-180°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = (-20 / 9 * GoDir - 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack1;
      Motor2 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack2;
      Motor3 = (20 / 9 * GoDir + 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack3;
      Motor4 = 100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack4;

      aa = 15; //動作を決める
    }
    else if (a == 50) // 0°~90°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = (20 / 9 * GoDir - 100) * MPOWER5 * MotorR1 + Kp * P - Kd * D;  //右前
      Motor2 = 100 * MPOWER5 * MotorR2 + Kp * P - Kd * D;                     //左前
      Motor3 = (-20 / 9 * GoDir + 100) * MPOWER5 * MotorR3 + Kp * P - Kd * D; //左後ろ
      Motor4 = -100 * MPOWER5 * MotorR4 + Kp * P - Kd * D;                    //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 60) // 90°~180°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100;

      Motor1 = 100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack1;                     //右前
      Motor2 = (-20 / 9 * GoDir + 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack2; //左前
      Motor3 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack3;                    //左後ろ
      Motor4 = (20 / 9 * GoDir - 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack4;  //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 70) // 0°~-90°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D;                    //右前
      Motor2 = (20 / 9 * GoDir + 100) * MPOWER5 * MotorR2 + Kp * P - Kd * D;  //左前
      Motor3 = 100 * MPOWER5 * MotorR3 + Kp * P - Kd * D;                     //左後ろ
      Motor4 = (-20 / 9 * GoDir - 100) * MPOWER5 * MotorR4 + Kp * P - Kd * D; //右後ろ

      aa = 15; //動作を決める
    }
    else if (a == 80) //-90°~-180°の方向に行きたいとき~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      if (a != b) //初期化
      {
        g = 0; //ボールを追いかける動作をしたことを記録する（ラインから離れる場合に必要になるため）
        b = a;
      };

      NowTime = millis();
      DoTime = NowTime + 100; // 100秒

      Motor1 = (-20 / 9 * GoDir - 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack1;
      Motor2 = -100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack2;
      Motor3 = (20 / 9 * GoDir + 300) * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack3;
      Motor4 = 100 * MPOWER5 * MotorR1 + Kp * P - Kd * D + BAtack4;

      aa = 15; //動作を決める
    }
  };

  if (serial != 0) // 0ではなく1だったら、シリアルモニターを表示
  {
    Serial.print("x:"); Serial.print(event.orientation.x);
    // Serial.print("  DIR20:"); Serial.print(DIR20);
    // Serial.print("  DIR90:"); Serial.print(DIR90);
    // Serial.print("  DIR180:"); Serial.print(DIR180);
    // Serial.print("  DIR270:"); Serial.print(DIR270);
    // Serial.print("  DIR340:"); Serial.print(DIR340);
    // Serial.print("  switch:"); Serial.print(digitalRead(SWICH));
    Serial.print("  LINE13:"); Serial.print(LINE13);
    // Serial.print("  LINEMtime:"); Serial.print(LINEMtime);
    // Serial.print("  MPlast1:"); Serial.print(MPlast1);
    // Serial.print("  M1:"); Serial.print(Motor1);
    // Serial.print("  M2:"); Serial.print(Motor2);
    // Serial.print("  M3:"); Serial.print(Motor3);
    // Serial.print("  M4:"); Serial.print(Motor4);
    // Serial.print("  DoTime:"); Serial.print(DoTime);
    Serial.print("  BAtack1"); Serial.print(BAtack1);
    Serial.print("  aa:"); Serial.print(aa);
    Serial.print("  h:"); Serial.print(h);
    Serial.print("  i:"); Serial.print(i);
    Serial.print("  Lpast:"); Serial.print(Lpast);
    Serial.print("  hata:"); Serial.print(hata);
    Serial.print("  b:"); Serial.print(b);
    // Serial.print("  c"); Serial.print(c);
    // Serial.print("  j:"); Serial.print(j);
    // Serial.print("  GoDir:"); Serial.print(GoDir);
    // Serial.print("  BValue:"); Serial.print(BValue);
    Serial.print("  BAngle:"); Serial.print(BAngle);
    Serial.print("  a:");Serial.println(a);
  };

  if (LineResetT < NowTime)
  {
    j = 0; //ラインセンサーリセット
  };
};