
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include  "robot.h" // подключаем файл с нужными функциями
#define BAUDRATE 115200
#define WHEEL_NUM  4
#define CMD_VEL_TIMEOUT 2000 //2 seconds

      
static float target_velocity[4] = {0.0, 0.0, 0.0, 0.0};
static float current_velocity[4] =  {0.0, 0.0, 0.0, 0.0};
static float current_effort[4] = {0.0, 0.0, 0.0, 0.0};
static float current_position[4] = {0.0, 0.0, 0.0, 0.0};
static uint32_t ir_sensors[28] = {0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0};
static float voltage[1] = {12};
static float*imu_arr;
uint32_t* buf_0;
uint32_t* buf_1;
uint32_t* buf_2;
uint32_t* buf_3;

int time_cmd_vel = 0;
int time_millis[7] = {0,0,0,0,0,0,0};
//HardwareTimer Timer(TIMER_CH4);//таймер для обновления информации приходящей из ROS-а
String inputString = "";
bool stringComplete = false;
int count = 0;
void setup() {
  init();//функция инициализирования Dxl устройств
  //Timer.stop();//инициализируем таймеры         
  //Timer.setPeriod(11000);// in microseconds
  //Timer.attachInterrupt(callback);//при прерывании вызывается функция handler_nh
  //Timer.start();
  delay(1000);
  Serial.begin(57600);
  inputString.reserve(500);
}

void loop(){
  if(voltage[0] > 9.5){
    if (stringComplete){
      inputString = "";
      stringComplete = false;
    }
    if(millis() - time_millis[0] >100)
    {
      int dt = millis() - time_millis[0];
      time_millis[0] = millis();
      move_os(target_velocity[0], -target_velocity[1], -target_velocity[2]);
      read_joint_state(current_velocity, current_effort);
      for(int i = 0; i<WHEEL_NUM; i++){
        if(abs(current_position[i]) < 10000)
          current_position[i] += current_velocity[i]*dt/1000;  
        else
          current_position[i] = 0;
      }
     sendSerial_float(current_velocity, WHEEL_NUM, "vel");
     sendSerial_float(current_effort, WHEEL_NUM, "eff");
     sendSerial_float(current_position, WHEEL_NUM, "pos");
    }
    if(millis() - time_millis[1] > 1000)
    {
      time_millis[1] = millis();
      voltage[0] = check_voltage();
    //  sendSerial_float(voltage, 1, "vol");
    }
    /*if(millis() - time_millis[2] > 45)
    {
      time_millis[2] = millis();
      imu_arr = read_IMU();
    //  sendSerial_float(imu_arr, 10, "imu");
    }*/
    if(millis() - time_millis[3] > 40){
        time_millis[3] = millis(); 
        if(count == 0)
          buf_0 = scan_front();
        if(count == 1)
          buf_1 = scan_left();
        if(count == 2)
          buf_2 = scan_right();
        if(count == 3){
          buf_3 = scan_back();
          for(int i = 0; i < 28; i++){
              if(i < 7)
                ir_sensors[i] = buf_0[i];
              if(i > 6 and i < 14)
                ir_sensors[i] = buf_1[i-7];
              if(i > 13 and i < 21)
                ir_sensors[i] = buf_2[i-14];
              if(i > 20 and i < 28)
                ir_sensors[i] = buf_3[i-21];
          }
          sendSerial_int(ir_sensors, 28, "light");
          count = -1;
       }
       count++;
    }
    if(millis()-time_cmd_vel > CMD_VEL_TIMEOUT){
      target_velocity[0]= 0;
      target_velocity[1]= 0;
      target_velocity[2] = 0;
    }
  }
  else{
    tone(BDPIN_BUZZER, 10, 250);
    delay(1000);
    noTone(BDPIN_BUZZER);
    delay(1000);
    move_os(0, 0, 0);
  }
  //delayMicroseconds(5);
}
void callback(){
  if (stringComplete){
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }
}
void sendSerial_float(float* data, int data_length, String name_){
  Serial.print(name_+",");
  for(int i = 0;i < data_length;i++){
    Serial.print(data[i]);
    if(i != (data_length-1))
      Serial.print(",");
  }
  Serial.println();
}
void sendSerial_int(uint32_t* data, int data_length, String name_){
  Serial.print(name_+",");
  for(int i = 0;i < data_length;i++){
    Serial.print((int)data[i]);
    if(i != (data_length-1))
      Serial.print(",");
  }
  Serial.println();
}

void serialEvent() {
  int i = 0;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == ','){
      target_velocity[i] = inputString.toFloat();
      inputString = "";
      i++;
    }
    else
      inputString += inChar;
    if (inChar == '\n') {
      target_velocity[i] = inputString.toFloat();
      stringComplete = true;
      time_cmd_vel = millis();
    }
  }
}
