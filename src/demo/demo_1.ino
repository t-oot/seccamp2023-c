#include "MPU9250.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <obniz.h>
#include <TinyGPSPlus.h>

const float mC = 261.626; // ド
const float mD = 293.665; // レ
const float mE = 329.628; // ミ
const float mF = 349.228; // ファ
const float mG = 391.995; // ソ
const float mA = 440.000; // ラ
const float mB = 493.883; // シ
const float nn = 0.0;

const float beep_wakeup[7] = {mE * 4, mA * 4, mB * 4, mA * 4, mE * 2, mE * 4, mB * 4}; // 起動音
const float beep_start[3] = {mC * 2, mD * 2, mE * 2};
const float beep_gps[1] = {mC * 1};
const float beep_gps2[1] = {mB * 1};
const float beep_end[3] = {mE * 2, mD * 2, mC * 2};
const float beep_error[5] = {mE * 4, mE * 4, mE * 4, mE * 4, mE * 4};

double goal_lat = 35.677039;
double goal_lon = 139.475214;
MPU9250 mpu;
TinyGPSPlus gps;
HardwareSerial hs(2);

const uint8_t pin_sda        = 21;
const uint8_t pin_scl        = 22;
const uint8_t pin_button     = 35;
const uint8_t pin_led        = 2;
const uint8_t pin_heat       = 15;
const uint8_t pin_speaker    = 12;
const uint8_t pin_sd_miso    = 19;
const uint8_t pin_sd_mosi    = 23;
const uint8_t pin_sd_sclk    = 18;
const uint8_t pin_sd_cs      = 5;
const uint8_t pin_motor_A[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const uint8_t pin_motor_B[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const uint8_t pin_gps_tx     = 16;
const uint8_t pin_gps_rx     = 17;

const int CHANNEL_A = 0; // PWMA
const int CHANNEL_B = 1; // PWMB
const int CHANNEL_C = 2; // Speaker

const int LEDC_TIMER_8_BIT    = 8;
const int LEDC_TIMER_13_BIT   = 13;
const int LEDC_BASE_FREQ_490  = 490;
const int LEDC_BASE_FREQ_5000 = 490;

volatile byte led_state = LOW;
volatile long interrupt_prev_ms = millis();

int gps_update = 0;
struct SensorVal {
  float roll;
  float pitch;
  float yaw;
  float lat;
  float lng;
  float head;
} sensorVal;

/** CanSatの状態遷移用の列挙型 */
enum {
  ST_STAND_BY = 0, // 待機
  ST_DRIVE,        // 目標地点へ走行
  ST_GOAL,         // 目標地点に到着
};

/** CanSatの状態遷移ステータス */
volatile int state = ST_STAND_BY;

/**
   setup関数
   最初に1回だけ実行される
*/
void setup() {
  Serial.begin(115200);
  Serial.println("Hello 100kinsat!");
  delay(2000);

  // 初期化処理
  obniz_init();
  pin_init();
  sd_init();
  mpu_init();
  gps_init();

  delay(2000);
  startUpdateMPUValTask();
  startUpdateGPSValTask();
  startSendObnizTask();

  beep(beep_wakeup, sizeof(beep_wakeup) / sizeof(float), 150);
}

/**
   loop関数
   繰り返し実行される
*/
void loop() {
  switch (state)
  {
    case ST_STAND_BY:
      Serial.println("*** ST_STAND_BY ***");
      stand_by();
      break;

    case ST_DRIVE:
      Serial.println("*** ST_DRIVE ***");
      drive();
      break;

    case ST_GOAL:
      Serial.println("*** ST_GOAL ***");
      goal();
      break;

    default:
      break;
  }
  delay(200);
}

/**
   check_goal()
   ゴール付近(10.0m以内)にいるか判定
*/
bool check_goal(double curr_lat, double curr_lon) {
  double distance = calc_distance(curr_lat, curr_lon, goal_lat, goal_lon);

  Serial.println("Check GOAL");
  if (distance < 10.0) {
    return true;
  }
  return false;
}

/**
 * 緯度・経度から距離の計算
 * ヒュベニの公式を利用：
https://qiita.com/yhornisse/items/eb98d06f1df087d283c9
 */
double calc_distance(double curr_lat, double curr_lon, double goal_lat, double goal_lon) {
  double R = 6371.0; // 地球の半径 (約6371km)

  double curr_lat_rad = radians(curr_lat);
  double curr_lon_rad = radians(curr_lon);
  double goal_lat_rad = radians(goal_lat);
  double goal_lon_rad = radians(goal_lon);

  double dlat = goal_lat_rad - curr_lat_rad;
  double dlon = goal_lon_rad - curr_lon_rad;

  double a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(curr_lat_rad) * cos(goal_lat_rad) * sin(dlon / 2.0) * sin(dlon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double distance = R * c * 1000.0;

  return distance;
}

/**
   calc_Bearing()
   緯度・経度から角度の計算
*/
double calc_Bearing(double curr_lat, double curr_lon, double goal_lat, double goal_lon) {
  double curr_lat_rad = radians(curr_lat);
  double curr_lon_rad = radians(curr_lon);
  double goal_lat_rad = radians(goal_lat);
  double goal_lon_rad = radians(goal_lon);

  double dlon = goal_lon_rad - curr_lon_rad;

  double y = sin(dlon) * cos(goal_lat_rad);
  double x = cos(curr_lat_rad) * sin(goal_lat_rad) - sin(curr_lat_rad) * cos(goal_lat_rad) * cos(dlon);
  double bearing = atan2(y, x);

  bearing = degrees(bearing);
  bearing = fmod(bearing + 360.0, 360.0);  // 0-360の範囲に修正

  return bearing;
}

void rotate_sec(int msec) {
  Serial.println("rotate");
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], HIGH);
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], HIGH);
  ledcWrite(CHANNEL_A, 255);
  ledcWrite(CHANNEL_B, 255);
  for (int i = 0; i < msec / 10; i++) {
    delay(10);
  }
  // 停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
}
// 指定した角度に移動する関数
void findN(int deg) {
  float currentAngle = abs(sensorVal.yaw); // 現在の角度を取得
  float diff = abs(deg - currentAngle); // 角度差を計算

  // 180度より大きな差を持つ場合、短い方向で回転させる
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  if (diff > 0) {
    // 右回転
    digitalWrite(pin_motor_A[0], LOW);
    digitalWrite(pin_motor_A[1], HIGH);
    digitalWrite(pin_motor_B[0], LOW);
    digitalWrite(pin_motor_B[1], HIGH);
  } else if (diff < 0) {
    // 左回転
    digitalWrite(pin_motor_A[0], HIGH);
    digitalWrite(pin_motor_A[1], LOW);
    digitalWrite(pin_motor_B[0], HIGH);
    digitalWrite(pin_motor_B[1], LOW);
  } else {
    // 既に目的の角度なので停止
    digitalWrite(pin_motor_A[0], LOW);
    digitalWrite(pin_motor_A[1], LOW);
    digitalWrite(pin_motor_B[0], LOW);
    digitalWrite(pin_motor_B[1], LOW);
    return;
  }

  // PWM値を設定

  // 指定の角度になるまで待つ
  while (abs(abs(sensorVal.head) - deg) > 10) { // 5度の誤差を許容
    float dif2 = abs(abs(sensorVal.head) - deg);
    if (dif2 < 50 || dif2 > 310) {
      ledcWrite(CHANNEL_A, 100);
      ledcWrite(CHANNEL_B, 100);
    } else if (dif2 < 100 || dif2 > 210) {
      ledcWrite(CHANNEL_A, 200);
      ledcWrite(CHANNEL_B, 200);
    } else {
      ledcWrite(CHANNEL_A, 255);
      ledcWrite(CHANNEL_B, 255);
    }
    delay(10);
  }

  // 停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
}

double get_dist(bool first) {
  float dist_m2 = 0;
  int ok = 0;

  if (first) {
    gps_update = 1;
  }
  int c = 0;
  while (true) {
    if ( gps_update == 0) {
      delay(100);
      Serial.println("checkGPS");
      c++;
      if (c > 30) {
        return -1;
      }
      continue;
    }
    double dist_m = calc_distance( sensorVal.lat, sensorVal.lng, goal_lat, goal_lon);
    Serial.print("gps_dist[m]:");
    Serial.println(dist_m);
    if (abs(dist_m - dist_m2) < 2) {
      ok++;
    } else {
      ok = 0;
    }
    if (ok > 5) {
      return dist_m;
    }
    dist_m2 = dist_m;
    delay(300);
  }

}

// 指定した角度に移動する関数
void moveToAngle(int targetAngle, int speed) {
  float currentAngle = abs(sensorVal.yaw); // 現在の角度を取得
  Serial.print("c");
  Serial.println(currentAngle);
  float diff = abs(targetAngle - currentAngle); // 角度差を計算
  float target_ = abs(targetAngle);

  // 180度より大きな差を持つ場合、短い方向で回転させる
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  if (diff > 0) {
    // 右回転
    digitalWrite(pin_motor_A[0], LOW);
    digitalWrite(pin_motor_A[1], HIGH);
    digitalWrite(pin_motor_B[0], LOW);
    digitalWrite(pin_motor_B[1], HIGH);
  } else if (diff < 0) {
    // 左回転
    digitalWrite(pin_motor_A[0], HIGH);
    digitalWrite(pin_motor_A[1], LOW);
    digitalWrite(pin_motor_B[0], HIGH);
    digitalWrite(pin_motor_B[1], LOW);
  } else {
    // 既に目的の角度なので停止
    digitalWrite(pin_motor_A[0], LOW);
    digitalWrite(pin_motor_A[1], LOW);
    digitalWrite(pin_motor_B[0], LOW);
    digitalWrite(pin_motor_B[1], LOW);
    return;
  }

  // PWM値を設定
  ledcWrite(CHANNEL_A, speed);
  ledcWrite(CHANNEL_B, speed);

  // 指定の角度になるまで待つ
  while (abs(abs(sensorVal.head) - target_) > 24) { // 5度の誤差を許容
    //Serial.println(sensorVal.head);
    //Serial.println(abs(abs(sensorVal.head) - target_));
    delay(10);
  }

  // 停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
}

/** ボタンの割り込み関数 */
void IRAM_ATTR onButton() {
  if (millis() > interrupt_prev_ms + 500) { // チャタリング防止
    led_state = !led_state;
    state = (state + 1) % 3;
    interrupt_prev_ms = millis();
  }
}

/**
   マルチタスクで実行する関数
   9軸センサの値の更新
*/
TaskHandle_t updateMPUValTaskHandle;
void updateMPUValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        prev_ms = millis();
      }
      sensorVal.roll = mpu.getRoll();
      sensorVal.pitch = mpu.getPitch();
      sensorVal.yaw = mpu.getYaw();
      float heading = 0;
      float mx = mpu.getMagX();
      float my =  mpu.getMagY();
      float mz = mpu.getMagZ();
      sensorVal.head = calculateAzimuth(mx, my, mz, sensorVal.pitch, sensorVal.roll);
      //Serial.println(heading);
    }
    delay(10);
  }
}

/**
   マルチタスクで実行する関数（updateMPUValTask）の開始
*/
void startUpdateMPUValTask() {
  xTaskCreatePinnedToCore(
    updateMPUValTask,
    "updateMPUValTask",
    8192,
    NULL,
    1,
    &updateMPUValTaskHandle,
    APP_CPU_NUM
  );
}

/**
   マルチタスクで実行する関数
   GPSセンサの値の更新
*/
TaskHandle_t updateGPSValTaskHandle;
void updateGPSValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    while (hs.available() > 0) {
      char c = hs.read();
      gps.encode(c);
      //Serial.print(c);
      if (gps.location.isUpdated()) {
        float lat = gps.location.lat();
        float lng = gps.location.lng();
        //Serial.println(sensorVal.lat - lat);
        //Serial.println(sensorVal.lng - lng);
        if (sensorVal.lat != lat || sensorVal.lng != lng) {
          // if (abs(sensorVal.lat - lat) > 0.000001 || abs(sensorVal.lng - lng) > 0.000001) {
          gps_update++;
          sensorVal.lat = lat;
          sensorVal.lng = lng;
          beep(beep_gps, sizeof(beep_gps) / sizeof(float), 150);
          if (calc_distance( sensorVal.lat, sensorVal.lng, goal_lat, goal_lon)<3) {
            state = ST_GOAL;
            break;
          }
          // }
        } else {
          beep(beep_gps2, sizeof(beep_gps2) / sizeof(float), 150);
        }
      }
    }
    delay(10);
  }
}

/**
   マルチタスクで実行する関数（updateGPSValTask）の開始
*/
void startUpdateGPSValTask() {
  xTaskCreatePinnedToCore(
    updateGPSValTask,
    "updateGPSValTask",
    8192,
    NULL,
    1,
    &updateGPSValTaskHandle,
    APP_CPU_NUM
  );
}

/**
   マルチタスクで実行する関数
   CanSat -> JavaScript へのデータ送信
*/
TaskHandle_t sendObnizTaskHandle;
void sendObnizTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    char message[512]; // JavaScriptへ送信するメッセージ用
    char roll_buf[16];
    char pitch_buf[16];
    char yaw_buf[16];
    char lat_buf[16];
    char lng_buf[16];
    char azi_buf[16];
    char dist_buf[16];

    if (obniz.isOnline()) {
      dtostrf(sensorVal.head, -1, 2, (char*)azi_buf);
      dtostrf(sensorVal.roll, -1, 2, (char*)roll_buf);
      dtostrf(sensorVal.pitch, -1, 2, (char*)pitch_buf);
      dtostrf(sensorVal.yaw, -1, 2, (char*)yaw_buf);
      dtostrf(sensorVal.lat, -1, 6, (char*)lat_buf);
      dtostrf(sensorVal.lng, -1, 6, (char*)lng_buf);
      dtostrf(calc_distance( sensorVal.lat, sensorVal.lng, goal_lat, goal_lon), -1, 6, (char*)dist_buf);
      sprintf(message, "%d,%s,%s,%s,%s,%s,%s,%s",
              state,
              roll_buf, pitch_buf, yaw_buf,
              lat_buf, lng_buf, azi_buf, dist_buf);
      obniz.commandSend((uint8_t*)message, strlen(message));
    }
    delay(500);
  }
}

/**
   マルチタスクで実行する関数（sendObnizTask）の開始
*/
void startSendObnizTask() {
  xTaskCreatePinnedToCore(
    sendObnizTask,
    "sendObnizTask",
    8192,
    NULL,
    1,
    &sendObnizTaskHandle,
    APP_CPU_NUM
  );
}

float calculateAzimuth(float magx, float magy, float magz, float pitch, float roll) {
  // ピッチとロールをラジアンに変換
  pitch = pitch * (PI / 180.0);
  roll = roll * (PI / 180.0);

  magx = magx / 32768.0f * 4800.0f;
  magy = magy / 32768.0f * 4800.0f;
  magz = magz / 32768.0f * 4800.0f;

  // 傾斜を考慮して地磁気の値を補正
  float magx_corrected = magx * cos(pitch) + magz * sin(pitch);
  float magy_corrected = magx * sin(roll) * sin(pitch) + magy * cos(roll) - magz * sin(roll) * cos(pitch);

  // 補正後の地磁気の値を使用して方位を計算
  float azimuth = atan2(-magy_corrected, magx_corrected) * (180.0 / PI);

  // 結果が負の場合は360度を加算して範囲を[0, 360)の間にする
  if (azimuth < 0) {
    azimuth += 360;
  }

  return azimuth;
}

/** 待機状態 */
void stand_by() {
  digitalWrite(pin_led, led_state);
}

/** 目標地点へ走行 */
void drive() {
  //  forward(255);
  //  delay(1000);
  //  stop();
  //  back(255);
  //  delay(1000);
  //  stop();
  //  findN(0);
  //  delay(5000);
  //  findN(180);
  //  delay(5000);
  //findN(0);
  bool first = true;
  while (true) {
    if (state == ST_GOAL)return;
    Serial.print("getCurrentGPS: ");
    double dist_m = get_dist(true);
    if (dist_m == -1) {
      Serial.println("error.retry");
      rotate_sec(random(1000, 5000));
      forward(255);
      delay(1000);
      stop();
      continue;
    }
    first = false;
    Serial.print("dist: ");
    Serial.println(dist_m);
    Serial.print("Rotate 5sec.....");
    rotate_sec(random(1000, 5000));
    //評価走行
    if (dist_m > 2) {
      forward(255);//fast
    } else if (dist_m > 1) {
      forward(200);//slow
    } else {
      forward(100);//very slow
    }
    delay(1000);
    gps_update = 0;
    delay(4000);
    stop();
    Serial.print("getnewGPS: ");
    double dist_m2 = get_dist(false);
    if (dist_m2 == -1) {
      Serial.println("err");
      continue;
    }
    Serial.print("OK\ndist2: ");
    Serial.println(dist_m2);
    if (dist_m2 < dist_m) {
      //近くなった
      Serial.println("correct direction");
      if (dist_m > 7) {
        forward(255);
      } else if (dist_m > 5) {
        forward(200);
      } else {
        if (dist_m < 3) {
          state = ST_GOAL;
          return;
        }
        forward(100);
      }
      delay(5000);
      gps_update = 0;
      delay(10000);
      stop();
    } else {
      //遠のいた
      Serial.println("wrong direction");
      continue;//やり直し
    }

  }

}

/** 目標地点に到着 */
void goal() {
  beep(beep_wakeup, sizeof(beep_wakeup) / sizeof(float), 150);
}

/** ObnizOSの初期化処理 */
void obniz_init() {
  obniz.start(NULL); // 引数にNULLを渡すとObnizOSのログがシリアルモニタに表示されなくなる
  // IOの管理をobnizOSから外す
  obniz.pinReserve(pin_sda);
  obniz.pinReserve(pin_scl);
  obniz.pinReserve(pin_button);
  obniz.pinReserve(pin_led);
  obniz.pinReserve(pin_heat);
  obniz.pinReserve(pin_speaker);
  obniz.pinReserve(pin_sd_miso);
  obniz.pinReserve(pin_sd_mosi);
  obniz.pinReserve(pin_sd_sclk);
  obniz.pinReserve(pin_sd_cs);
  for (int i = 0; i < 3; i++) {
    obniz.pinReserve(pin_motor_A[i]);
    obniz.pinReserve(pin_motor_B[i]);
  }
  obniz.pinReserve(pin_gps_tx);
  obniz.pinReserve(pin_gps_rx);
}

/** GPIOの初期化処理 */
void pin_init() {
  pinMode(pin_button, INPUT);
  attachInterrupt(pin_button, onButton, FALLING);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_heat, OUTPUT);
  digitalWrite(pin_heat, LOW); // 電熱線のピンはLOWにしておく
  pinMode(pin_speaker, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(pin_motor_A[i], OUTPUT);
    pinMode(pin_motor_B[i], OUTPUT);
  }
  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcAttachPin(pin_motor_A[2], CHANNEL_A);
  ledcAttachPin(pin_motor_B[2], CHANNEL_B);
}

/** SDカードの初期化処理 */
void sd_init() {
  if (!SD.begin()) {
    Serial.println("Card mount failed.");
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  writeFile(SD, "/100kinsat.txt", "Hello 100kinSAT!!!");
}

/** 9軸センサの初期化処理 */
void mpu_init() {
  Wire.begin();
  delay(2000);

  // if (!mpu.setup(0x69)) { // サイはI2Cアドレスが違う
  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  mpu.setMagneticDeclination(-7.49); // 磁気偏角の設定（府中駅: -7.49）
  mpu.selectFilter(QuatFilterSel::MADGWICK); // フィルターの設定
  mpu.setFilterIterations(10);

  mpu.verbose(true);

  // 加速度/ジャイロセンサのキャリブレーション
  // キャリブレーション中はCanSatを平らな地面で静止させておく
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  mpu.calibrateAccelGyro();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  delay(1000);

  // 地磁気センサのキャリブレーション
  // キャリブレーション中はCanSatをぐるぐる回転させる
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  //mpu.calibrateMag();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  mpu.verbose(false);
}

String ch_readline() {

  String received = "";
  char ch;
  unsigned long startTime = millis();

  while (millis() - startTime < 1000) {
    if (hs.available()) {
      ch = hs.read();
      if (ch == '\n') {
        return received;
      }
      else {
        received += ch;
      }
    }
  }
  return received;
}
/** GPSセンサの初期化処理 */
void gps_init() {
  hs.begin(9600);
  //https://www.yuden.co.jp/wireless_module/document/weather_appli/en/TY_GPS_GYSFDMAXB_NMEAPacketFormat_V1.0E_20170411.pdf
  //https://nmeachecksum.eqth.net/
  //通信速度
  //  hs.print("$PMTK251,115200*1F\r\n");
  //  delay(500);
  //  hs.flush();
  //  hs.end();
  //  hs.begin(115200);
  //測位間隔 250ms
  for (int i = 0; i < 2; i++) {
    //hs.print("$PMTK300,500,0,0,0,0*28\r\n");
    hs.print("$PMTK300,250,0,0,0,0*2A\r\n");
    Serial.println(ch_readline());
    hs.print("$PMTK300,250,0,0,0,0*2A\r\n");
    //hs.print("$PMTK300,500,0,0,0,0*28\r\n");
    Serial.println(ch_readline());
  }
  hs.print("$PMTK220,250*29\r\n");
  Serial.println(ch_readline());
  //hs.print("$PMTK300,1000,0,0,0,0*1C\r\n");

  //みちびきON
  hs.print("$PMTK351,1*28\r\n");
  Serial.println(ch_readline());
  //出力センテンス選択
  //hs.print("$PMTK314,0,1,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n");
  //Serial.println(ch_readline());
  Serial.println("GPS begin");
}

/** 前進 */
void forward(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CCW，反時計回り）
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(pin_motor_B[1], LOW);
  digitalWrite(pin_motor_B[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 後退 */
void back(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CW，時計回り）
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_A[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 停止 */
void stop() {
  // 左モータ停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}

/** SDカードに新規書き込みする */
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

/** SDカードに追記する */
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

/** スピーカーから音を鳴らす */
void beep(const float *mm, int m_size, int t_ms) {
  for (int i = 0; i < m_size; i++) {
    tone(pin_speaker, mm[i], t_ms);
  }
  noTone(pin_speaker);
}

void tone(int pin, int freq, int t_ms) {
  ledcSetup(CHANNEL_C, LEDC_BASE_FREQ_5000, LEDC_TIMER_13_BIT);
  ledcAttachPin(pin, CHANNEL_C);
  ledcWriteTone(CHANNEL_C, freq);

  delay(t_ms);
}

void noTone(int pin) {
  ledcWriteTone(CHANNEL_C, 0.0);
}
