////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu;

#include <EEPROM.h>
#define EEPROM_size 28 // ax, ay, az error, gx,gy,gz error

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s
Kalman kalmanY; // Create the Kalman instances
double p_kf=0;
uint32_t timer_kf;
////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// ENCODER //////////////////////////////////////////////////////////////
#define enc_1A_pin 39
#define enc_1B_pin 34
#define enc_2A_pin 35
#define enc_2B_pin 32

long count_1=0, count_2=0; 
int enc_1_speed=0, enc_2_speed=0, previous_speed=0;
float a=0.5; // weight of previous value
////////////////////////////////////////////////////////////// ENCODER //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int update_interval=200; // time interval in ms for updating panel indicators
unsigned long last_time=0; // time of last update
char data_in; // data received from serial link
int pitch_sen=100, yaw_sen=100;
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// CONTROL //////////////////////////////////////////////////////////////
float KP_stand=280*0.6;
float KD_stand=-3000*0.6; 

float KP_speed=-0.060;
float KI_speed=KP_speed/200;

float KP_yaw=16; 
float KD_yaw=-2300; 
float med_angle=-1.0;
// int M1_fric=540, M2_fric=470;
// int M1_fric=550, M2_fric=550;
int target_speed=0,previous_target_speed, err_sum_speed, target_yaw_speed=0; 
int target_speed_BT, target_yaw_speed_BT;
float stand_PD_output, speed_PI_output, yaw_PD_output;

////////////////////////////////////////////////////////////// CONTROL //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// MODE //////////////////////////////////////////////////////////////
#define KEY1 33
#define KEY2 25
#define KEY3 26
int mode=1; // 1=BT, 2=BT+USS, 3=follow
unsigned long mode_change_time=0;
////////////////////////////////////////////////////////////// MODE //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Voltage monitor //////////////////////////////////////////////////////////////
#define Vb_pin 12
#define voltage_constant 0.0051916499
float Vb;
// void read_voltage_loop(){Vb=voltage_constant*analogRead(Vb_pin);} // volts
////////////////////////////////////////////////////////////// Voltage monitor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Start music //////////////////////////////////////////////////////////////
// const int buzzer_pin = 13;
// int freq[] = {
//   125,// 你
//   135,138,141,135,129,124,119,114,109,104,99,94,89,84,79,74,// 干
//   160,163,166,169,172,175,178,181,184,187,190,193,// 嘛
//   110, // 害
//   80, // 害
//   30}; // 哟

// int note_duration[] = {
//   350,
//   30, 30, 80, 30, 30, 30, 30, 30, 30,30,30,30,30,30,30 , 150,
//   20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,800,
//   300,
//   900,
//   300};

// void start_music(){
//   pinMode(buzzer_pin, OUTPUT);
//   for (int i = 0; i < 32; i++) {
//     int noteDuration = note_duration[i];
//     tone(buzzer_pin, freq[i], noteDuration);
//     noTone(buzzer_pin); 
//   }
//   pinMode(buzzer_pin, INPUT);
//   delay(5000);
// }
////////////////////////////////////////////////////////////// Start music //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////
#define M1_pin 17 // B6
#define M1_PWM_CH 0
#define M1_PWM_res 12 // 
#define M1_PWM_freq 10000

#define M2_pin 16 // B7
#define M2_PWM_CH 1
#define M2_PWM_res 12 // 
#define M2_PWM_freq 10000

#define AIN1 19
#define AIN2 5
#define BIN1 15
#define BIN2 0

#define STBY_pin 4

int M1_speed=0, M2_speed=0; // 0~+-4096

#define S1_pin 23
#define S2_pin 18
const int up_time=500, down_time=200;
const int S1_down_angle=500, S1_up_angle=800;
const int S2_down_angle=2500, S2_up_angle=2000;
unsigned long previous_move_time=0;
bool up_down=1,servo_ena=0,S1_state=1,S2_state=1;
unsigned long servo_change_time=0;
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Ultra sonic Sensor //////////////////////////////////////////////////////////////
#define trig_pin 14
#define echo_pin 27
#define sound_speed 0.034 // cm/us
#define target_dist 20 // cm
unsigned long t0,t1;
float distance;
bool echoed_flag=1;
////////////////////////////////////////////////////////////// Ultra sonic Sensor //////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(460800);
  
  build_panel(); // BT setup

  ultra_sonic_sensor_setup();

  mpu6050_setup();

  mode_setup();

  encoder_setup();

  motor_output_setup();

  // start_music();

  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  Serial.print("Set up ends");

}

void loop() {
  read_bluetooth_data();

  mode_loop();
  send_bluetooth_data();

  loop_time_holder();

  // read_encoder_loop(); // called in control

  // read_mpu6050_angle_loop(); // called in control
  
  control_loop();

  motor_output_loop();

  Serial.println();
}

////////////////////////////////////////////// TIME //////////////////////////////////////////////
unsigned long loop_start_time=0;
const int loop_time=10000; // micro seconds
void loop_time_holder(){
  // if(micros() - loop_start_time > (loop_time+50)) digitalWrite(2, HIGH);
  // else digitalWrite(2, LOW);

  // unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  loop_start_time = micros(); 
}

unsigned long loop2_start_time=0;
////////////////////////////////////////////// TIME //////////////////////////////////////////////
////////////////////////////////////////////// IMU //////////////////////////////////////////////
void mpu6050_setup(){ 
  Serial.flush();
  pinMode(2,OUTPUT);
  pinMode(KEY1,INPUT);
  delay(2500);
  // Serial.println("wait for serial port input");
  // delay(5000); // wait for serial port input
  mpu6050_start();

  EEPROM.begin(EEPROM_size);

  // IMU_calibration(); 

  if (digitalRead(KEY1)==LOW){ // if pushed
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    delay(5000);
    IMU_calibration(); 
  } else {
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    EEPROM.get(0, ax_error);
    EEPROM.get(4, ay_error);
    EEPROM.get(8, az_error);
    EEPROM.get(12, wx_error);
    EEPROM.get(16, wy_error);
    EEPROM.get(20, wz_error);
    EEPROM.get(24, med_angle);
  }
  
  delay(2000);

  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;

  double pitch=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));

  kalmanY.setAngle(pitch); // Set starting angle

  timer_kf = micros();
}

void mpu6050_start(){
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void read_accel_gyro_raw(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax_float=a.acceleration.x;
  ay_float=a.acceleration.y;
  az_float=a.acceleration.z;

  wx=g.gyro.x;
  wy=g.gyro.y;
  wz=g.gyro.z;
}

void IMU_calibration() {
  digitalWrite(2,HIGH); 
  Serial.println("Start IMU calibration. ");
  // acceleration error
  int num_of_loop=1000; // number of sample
  int i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    ax_error=ax_error+ax_float; // unit: m/s^2
    ay_error=ay_error+ay_float;
    az_error=az_error+az_float;
    i++;
  }
  ax_error=ax_error/num_of_loop; // unit: raw data
  ay_error=ay_error/num_of_loop;
  az_error=(az_error/num_of_loop-9.81); // should measure 1g when stationary. chip face is -z
  EEPROM.put(0, ax_error);
  EEPROM.put(4, ay_error);
  EEPROM.put(8, az_error);
  
  // angular velocity error
  i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    wx_error=wx_error+wx; // unit: rad/sec
    wy_error=wy_error+wy;
    wz_error=wz_error+wz;
    i++;
  }
  wx_error=wx_error/num_of_loop; // rad/sec
  wy_error=wy_error/num_of_loop;
  wz_error=wz_error/num_of_loop;
  EEPROM.put(12, wx_error);
  EEPROM.put(16, wy_error);
  EEPROM.put(20, wz_error);
  EEPROM.commit();
  delay(500);
  Serial.println("EEPROM wrote");
  delay(2000);

  Serial.println("Minus these errors when converting from int16_t to float. ");
  Serial.println("Acceleration error (m/s^2): ");
  Serial.print("ax_error: ");
  Serial.println(ax_error);
  Serial.print("ay_error: ");
  Serial.println(ay_error);
  Serial.print("az_error: ");
  Serial.println(az_error);
  Serial.println();

  Serial.println("Angular velocity error (rad/s): ");
  Serial.print("wx_error: ");
  Serial.println(wx_error);
  Serial.print("wy_error: ");
  Serial.println(wy_error);
  Serial.print("wz_error: ");
  Serial.println(wz_error);

  Serial.println("Accel and gyro Calibration finished.");
  Serial.println();
  digitalWrite(2,LOW); delay(100);
}

void read_mpu6050_angle_loop(){
  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;
  wx-=wx_error;
  wy-=wy_error;
  wz-=wz_error;

  double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
  timer_kf = micros();
  
  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float)))*180/M_PI;
  p_kf = kalmanY.getAngle(pitch, wx*180/M_PI, dt); // pitch
  // Serial.print(-50); // To freeze the lower limit
  // Serial.print(" ");
  // Serial.print(50); // To freeze the upper limit
  // Serial.print(" ");

  // Serial.print("Pitch: ");
  // Serial.print(p_kf); Serial.print("\t");
  Serial.print("p_kf:");Serial.print(p_kf); // Serial.print("\t");
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////
////////////////////////////////////////////// ENCODER //////////////////////////////////////////////
// hw_timer_t * timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// void IRAM_ATTR onTimer() {
//   portENTER_CRITICAL_ISR(&timerMux);
//   enc_1_speed=count_1*10;
//   enc_2_speed=count_2*10;
//   // Serial.print(count_1*10); Serial.print("\t"); 
//   // Serial.print(count_2*10); Serial.print("\t"); 
//   count_1=0;
//   count_2=0;
//   portEXIT_CRITICAL_ISR(&timerMux);
// }

void read_encoder_loop(){
  enc_1_speed=count_1*10;
  enc_2_speed=count_2*10;
  count_1=0;
  count_2=0;
}

void encoder_setup(){
  // timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 10000, true); // 5000 microsecond interrput
  // timerAlarmEnable(timer);

  // pinMode(0,OUTPUT); digitalWrite(0,0);
  // pinMode(4,OUTPUT); digitalWrite(4,0);
  // pinMode(17,OUTPUT); digitalWrite(17,0);
  // pinMode(16,OUTPUT); digitalWrite(16,0);

  pinMode(enc_1A_pin, INPUT_PULLUP);
  pinMode(enc_1B_pin, INPUT_PULLUP);
  pinMode(enc_2A_pin, INPUT_PULLUP);
  pinMode(enc_2B_pin, INPUT_PULLUP);

  attachInterrupt(enc_1A_pin,enc_1_A,CHANGE);
  attachInterrupt(enc_1B_pin,enc_1_B,CHANGE);
  attachInterrupt(enc_2A_pin,enc_2_A,CHANGE);
  attachInterrupt(enc_2B_pin,enc_2_B,CHANGE);
}

void enc_1_A(){
  if (digitalRead(enc_1A_pin) == digitalRead(enc_1B_pin)) count_1++;  
  else count_1--;
  // Serial.print("M1 count: ");
  // Serial.println(count_1);
}

void enc_1_B(){
  if (digitalRead(enc_1A_pin) == digitalRead(enc_1B_pin)) count_1--;  
  else count_1++;
  // Serial.print("M1 count: ");
  // Serial.println(count_1);
}

void enc_2_A(){
  if (digitalRead(enc_2A_pin) == digitalRead(enc_2B_pin)) count_2++;
  else count_2--;
  // Serial.print("M2 count: ");
  // Serial.println(count_2);
}

void enc_2_B(){
  if (digitalRead(enc_2A_pin) == digitalRead(enc_2B_pin)) count_2--;
  else count_2++;
  // Serial.print("M2 count: ");
  // Serial.println(count_2);
}
////////////////////////////////////////////// ENCODER //////////////////////////////////////////////
////////////////////////////////////////////// CONTROL //////////////////////////////////////////////
// unsigned long previous_KP_time=0;
// float scale_factor=1; 
void control_loop(){
    // Serial.print("TS:");Serial.print(target_speed); Serial.print("\t");
    // Serial.print("KP:");Serial.print(KP_speed,3); Serial.print("\t");
    // Serial.print("KI:");Serial.print(KI_speed,7); Serial.print("\t");
    read_encoder_loop();
    speed_PI(target_speed, enc_1_speed, enc_2_speed);
    previous_target_speed=target_speed;
    // Serial.print("KP:");Serial.print(KP_stand); Serial.print("\t");
    // Serial.print("KD:");Serial.print(KD_stand); Serial.print("\t");
    read_mpu6050_angle_loop();
    stand_PD(speed_PI_output+med_angle, p_kf, wx); //speed_PI_output+
    
    // Serial.print("TY:");Serial.print(target_yaw_speed); Serial.print("\t");
    // Serial.print("KP:");Serial.print(KP_yaw); Serial.print("\t");
    // Serial.print("KD:");Serial.print(KD_yaw); Serial.print("\t");
  yaw_PD(target_yaw_speed,wz);

  M1_speed=stand_PD_output-yaw_PD_output;
  M2_speed=stand_PD_output+yaw_PD_output;

  // M1_speed=stand_PD_output+speed_PI_output-yaw_PD_output;
  // M2_speed=stand_PD_output+speed_PI_output+yaw_PD_output;
  // 临时调试,旋转方向
  // M1_speed=speed_PI_output-yaw_PD_output;
  // M2_speed=speed_PI_output+yaw_PD_output;

  // if (M1_speed>0) M1_speed+=M1_fric;
  // else if (M1_speed<0) M1_speed-=M1_fric;
  // if (M2_speed>0) M2_speed+=M2_fric;
  // else if (M2_speed<0) M2_speed-=M2_fric;

  // if (M1_speed>0) M1_speed=constrain(M1_speed, M1_fric,4096);
  // else if (M1_speed<0 ) M1_speed=constrain(M1_speed,-4096,-M1_fric);
  // if (M2_speed>0) M1_speed=constrain(M2_speed, M2_fric,4096);
  // else if (M2_speed<0 ) M2_speed=constrain(M2_speed,-4096,-M2_fric);

  M1_speed=constrain(M1_speed, -4096,4096);
  M2_speed=constrain(M2_speed, -4096,4096);

  // Serial.print("M1:");Serial.print(M1_speed); Serial.print("\t");
  // Serial.print("M2:");Serial.print(M2_speed); Serial.print("\t");
}

float err_stand=0,err_stand_pre=0, err_stand_sum=0;
void stand_PD(float target, float actual, float ang_speed){
  float err=target-actual;

  float KP_out, KD_out; //KI_out;
  KP_out=KP_stand*err;
  KD_out=KD_stand*ang_speed;

  stand_PD_output=KP_out+KD_out; // +KI_out;
  // stand_PD_output=KP_stand*err+KD_stand*ang_speed+KI_stand*err_stand_sum*KI_flag;
  // Serial.print("KP_out:");Serial.print(KP_out); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out); Serial.print("\t");
  // Serial.print("KD_out:");Serial.print(KD_out); Serial.print("\t");
}

// float pre_speed=0;
void speed_PI(float target, int v1, int v2){
  // v1/v2: wheel 1/2 ang speed; target:target ang speed
  int speed=(v1+v2)*(1-a)+previous_speed*a;
  previous_speed=speed;
  Serial.print("wheel:");Serial.print(speed); Serial.print(" ");
  // pre_speed=speed;
  float err=target-speed;
  // Serial.print("Speed err:");Serial.print(err); Serial.print("\t");
  err_sum_speed+=err;
  if (err_sum_speed>10000)  err_sum_speed=10000;
  if (err_sum_speed<-10000)  err_sum_speed=-10000;
  if (previous_target_speed != target) err_sum_speed=0;
  if (abs(p_kf)>55) err_sum_speed=0;

  float KP_out,KI_out; // , KD_out;
  KP_out=KP_speed*err;
  KI_out=KI_speed*err_sum_speed;
  // KD_out=KD_speed*(err-pre_err_speed);
  speed_PI_output=KP_out+KI_out; //+KD_out;

  // speed_PI_output=KP_speed*err+KI_speed*err_sum_speed+KD_speed*(err-pre_err_speed);
  // pre_err_speed=err;
  // Serial.print("speed_PI_output:");Serial.print(speed_PI_output); Serial.print("\t");
  // Serial.print("KP_out:");Serial.print(KP_out,5); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out,5); Serial.print("\t");
}

void yaw_PD(float target, float ang_speed){
  if (target==0){
    yaw_PD_output=KD_yaw*ang_speed;
  } else {
    yaw_PD_output=KP_yaw*target;
  }
  
}
////////////////////////////////////////////// CONTROL //////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
void read_bluetooth_data(){
  // fromt serial monitor
  if (Serial.available()){
    data_in=Serial.read();  //Get next character
    if(data_in=='A'){KP_stand=float(Serial.parseInt());}
    if(data_in=='B'){KD_stand=float(Serial.parseInt());}
    if(data_in=='C'){KP_speed=float(Serial.parseInt())/1000;KI_speed=KP_speed/200;}
    if(data_in=='D'){KI_speed=float(Serial.parseInt())/1000000;}
    if(data_in=='E'){KP_yaw=float(Serial.parseInt());}
    if(data_in=='F'){KD_yaw=float(Serial.parseInt());}
    if(data_in=='N'){med_angle=float(Serial.parseInt())/100;}
    if(data_in=='a'){a=float(Serial.parseInt())/100;}
  }

  // From bluetooth
  if (SerialBT.available()){
    data_in=SerialBT.read();  //Get next character
    
    // from terminal
    if(data_in=='A'){KP_stand=float(SerialBT.parseInt());}
    else if(data_in=='B'){KD_stand=float(SerialBT.parseInt());}
    else if(data_in=='C'){KP_speed=float(SerialBT.parseInt())/1000;KI_speed=KP_speed/200;}
    else if(data_in=='D'){KI_speed=float(SerialBT.parseInt())/1000000;}
    else if(data_in=='E'){KP_yaw=float(SerialBT.parseInt());}
    else if(data_in=='F'){KD_yaw=float(SerialBT.parseInt());}
    else if(data_in=='N'){med_angle=p_kf;EEPROM.put(24, med_angle);EEPROM.commit();}
    else if(data_in=='L'){servo_ena=!servo_ena;}

    // from others
    else if(data_in=='M'){mode++;} //Button change mode
    // if(data_in=='R'){pitch_sen=SerialBT.parseInt();} // pitch/speed sensitivity
    // if(data_in=='L'){yaw_sen=SerialBT.parseInt();} // yaw sensitivity
    
    if(data_in=='P'){ // right joy stick
      while(true){
        if (SerialBT.available()){
          data_in=SerialBT.read();  //Get next character
          if(data_in=='Y') target_speed_BT=-SerialBT.parseInt()*1.2; // encoder speed
          if(data_in=='P') break; // End character
        }
      }
    }

    if(data_in=='W'){ // left joy stick
      while(true){
        if (SerialBT.available()){
          data_in=SerialBT.read();  //Get next character
          if(data_in=='X') target_yaw_speed_BT=-SerialBT.parseInt();
          if(data_in=='W') break; // End character
        }
      }
    }

  }
}

void send_bluetooth_data(){
  unsigned long t=millis();
  if ((t-last_time)>update_interval){
    String temp;
    last_time=t;
    // voltage monitor
    Vb=voltage_constant*analogRead(Vb_pin);
    // Serial.print("Vb: "); Serial.print(Vb); Serial.print("\t");
    SerialBT.print("*V"+String(Vb)+"*");

    // mode indicator
    if (mode==1) temp="BT";
    else if (mode==2) temp="BT+USS";
    else if (mode==3) temp="Follow";
    SerialBT.print("*M"+temp+"*");
    int speed_temp=(enc_1_speed+enc_2_speed)*(1-a)+a*previous_speed;
    // Serial.print("enc_1+2===:");Serial.print(enc_1_speed+enc_2_speed); Serial.print(" ");
    // // Serial.print("enc_2_speed:");Serial.print(enc_2_speed); Serial.print(" ");
    // Serial.print("a:");Serial.print(a); Serial.print(" ");
    // Serial.print("previous_speed:");Serial.print(previous_speed); Serial.print(" ");
    // Serial.print("speed_temp:");Serial.print(speed_temp); Serial.print(" ");
    SerialBT.print("*H"+String(speed_temp)+"*"); // speed
    SerialBT.print("*I"+String(med_angle)+"*");
    SerialBT.print("*G"+String(p_kf)+"*");
    SerialBT.print("*F"+String(distance)+"*");
  }
}

void build_panel(){
  SerialBT.begin(460800);
  SerialBT.println("*.kwl");
  SerialBT.println("clear_panel()");
  SerialBT.println("set_grid_size(12,9)");
  SerialBT.println("add_text_box(0,1,3,C,M2,245,240,245,)");
  SerialBT.println("add_text_box(6,1,3,C,Ctrl Mode,245,240,245,)");
  SerialBT.println("add_text_box(3,1,3,C,Vb,245,240,245,)");
  SerialBT.println("add_text_box(9,1,3,C,M1,245,240,245,)");
  SerialBT.println("add_text_box(0,2,2,L,0,245,240,245,G)");
  SerialBT.println("add_text_box(10,2,2,L,0,245,240,245,F)");
  SerialBT.println("add_text_box(0,0,3,C,,245,240,245,I)");
  SerialBT.println("add_text_box(9,0,3,C,,245,240,245,H)");
  SerialBT.println("add_text_box(3,0,3,C,,245,240,245,V)");
  SerialBT.println("add_text_box(6,0,3,C,BT,245,240,245,M)");
  SerialBT.println("add_button(7,6,1,M,)");
  SerialBT.println("add_slider(1,9,3,0,100,0,A,A,0)");
  SerialBT.println("add_slider(0,4,4,0,100,15,S,,0)");
  SerialBT.println("add_slider(1,8,2,30,200,30,L,,0)");
  SerialBT.println("add_slider(8,8,2,30,200,164,R,,0)");
  SerialBT.println("add_free_pad(8,5,-255,255,0,0,P,P)");
  SerialBT.println("add_free_pad(1,5,-255,255,0,0,W,W)");
  SerialBT.println("add_send_box(4,2,5,S50,,)");
  SerialBT.println("set_panel_notes(-,,,)");
  SerialBT.println("run()");
  SerialBT.println("*");
}
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////

void motor_output_setup(){
  ledcAttachPin(M1_pin,M1_PWM_CH);
  ledcSetup(M1_PWM_CH,M1_PWM_freq,M1_PWM_res);

  ledcAttachPin(M2_pin,M2_PWM_CH);
  ledcSetup(M2_PWM_CH,M2_PWM_freq,M2_PWM_res);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY_pin,OUTPUT);
  digitalWrite(STBY_pin,HIGH);

  // servo
  pinMode(S1_pin, OUTPUT);
  pinMode(S2_pin, OUTPUT);
}

void motor_output_loop(){
  if (abs(p_kf)>55) {M1_speed=0; M2_speed=0;}

  if (M1_speed>=0){
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(M1_PWM_CH, abs(M1_speed));
  } else if (M1_speed<0){
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(M1_PWM_CH, abs(M1_speed));
  }

  if (M2_speed>=0){
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(M2_PWM_CH, abs(M2_speed));
  } else if (M2_speed<0){
    digitalWrite(BIN2, LOW); digitalWrite(BIN1, HIGH);
    ledcWrite(M2_PWM_CH, abs(M2_speed));
  }

  if (servo_ena){
    digitalWrite(2,HIGH);
    servo_output();
  } else digitalWrite(2,LOW);
}

void servo_output(){
    int S1_angle, S2_angle;
    if (up_down){                                 // if is at up position
      if ((millis()-previous_move_time)>up_time){ // if reached up position stay time
        S1_angle=S1_down_angle;                   // set angle to down angle
        S2_angle=S2_up_angle;
        up_down=0;                                // set position to down
        previous_move_time=millis();
      } else {                                    // else stay same
        S1_angle=S1_up_angle; 
        S2_angle=S2_down_angle; 
      }                             
    } else if (!up_down){                         // if is at down position
      if ((millis()-previous_move_time)>down_time){
        S1_angle=S1_up_angle;
        S2_angle=S2_down_angle;
        up_down=1;
        previous_move_time=millis();
      } else {
        S1_angle=S1_down_angle;
        S2_angle=S2_up_angle;
      }
    } 
    while ((micros()- loop2_start_time)<loop_time){} 
    loop2_start_time = micros(); 
    digitalWrite(S1_pin, HIGH);
    digitalWrite(S2_pin, HIGH);
    unsigned long S1_end_time=loop2_start_time+S1_angle;
    unsigned long S2_end_time=loop2_start_time+S2_angle;

    S1_state=1; S2_state=1;
    while(S1_state || S2_state){
      unsigned long now_time=micros();
      if (S1_end_time<=now_time){
        digitalWrite(S1_pin, LOW);
        S1_state=0;
      } else S1_state=1;
      if (S2_end_time<=now_time){
        digitalWrite(S2_pin, LOW); 
        S2_state=0;
      } else S2_state=1;
    }
}
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////
////////////////////////////////////////////////////////////// Ultrasonic Sensor //////////////////////////////////////////////////////////////
void ultra_sonic_sensor_setup(){
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT); 
  attachInterrupt(echo_pin,read_distance,FALLING);
}

void read_distance(){
  t1=micros();
  echoed_flag=1;
}

void ultra_sonic_sensor_loop(){
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Sets the trig_pin on HIGH state for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  echoed_flag=0;
  t0=micros();
}

////////////////////////////////////////////////////////////// Ultrasonic Sensor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// MODE //////////////////////////////////////////////////////////////
void mode_setup(){
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);
  pinMode(KEY3, INPUT);
  attachInterrupt(KEY1,fc1,FALLING);
  attachInterrupt(KEY2,fc2,FALLING);
  attachInterrupt(KEY3,fc3,FALLING);
}
void fc1(){
  long t=millis();
  if ((t-mode_change_time)>300){
    mode++;
    mode_change_time=millis();
  }
}
void fc2(){
  med_angle=p_kf;
}
void fc3(){
  long t=millis();
  if ((t-servo_change_time)>300){
    servo_ena=!servo_ena;
    servo_change_time=millis();
  }
}
void mode_loop(){
  if (mode==4) {
    mode=1;
    target_speed_BT=0;
    target_yaw_speed_BT=0;
  }
  Serial.print("mode: "); Serial.print(mode); Serial.print("\t");
  
  if (mode!=1){ // BT control + ultra sonic sensor
    if (echoed_flag){ // if echoed 
      distance=(t1-t0)*sound_speed/2-8;
      // Serial.print("dist: "); Serial.print(distance); Serial.print("\t");
      // if (mode==2 && distance<25 && target_speed_BT>0) target_speed_BT*=0.3;
      if (mode==2 && distance<15){
        target_speed_BT=constrain(target_speed_BT,-255,0);
        if (distance<15) target_yaw_speed_BT=0;
      } else if (mode==3 && distance<50){
        if ((abs(distance-target_dist)>2) && abs(p_kf)<15) target_speed_BT=(distance-target_dist)*10;
      }
      ultra_sonic_sensor_loop(); // if echoed (found distance),start next trig and echo
    }
  } 

  // Serial.print("TS bef:");Serial.print(target_speed_BT); Serial.print("\t");
  // Serial.print("TY bef:");Serial.print(target_yaw_speed_BT); Serial.print("\t");
  float max_speed=2*abs(target_speed_BT)+abs(target_yaw_speed_BT)*2;
  // Serial.print("max before:");Serial.print(max_speed); Serial.print("\t");

  float scale_factor=0;
  if (abs(max_speed)>700){
    scale_factor=700.0f/max_speed;
    target_speed=target_speed_BT*scale_factor;
    target_yaw_speed=target_yaw_speed_BT*scale_factor;
  } else {
    target_speed=target_speed_BT;
    target_yaw_speed=target_yaw_speed_BT;
  }
  // Serial.print(" TS aft:");Serial.print(target_speed); Serial.print("\t");
  // Serial.print("TY aft:");Serial.print(target_yaw_speed); Serial.print("\t");
  // Serial.print("max aft:");Serial.print(max_speed); Serial.print("\t");
  // Serial.print("scale_factor:");Serial.print(scale_factor); Serial.print("\t");
  
}
////////////////////////////////////////////////////////////// MODE //////////////////////////////////////////////////////////////