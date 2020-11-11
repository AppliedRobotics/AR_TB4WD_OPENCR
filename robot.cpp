#include "robot.h"
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" 
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
#define BAUDRATE  1000000  // скорость опроса датчиков
#define DXL_1    1  // передний левый
#define DXL_2    2  // передний правый
#define DXL_3    3  // задний левый
#define DXL_4    4  // задний правый
#define Sensor_Front_ID  10 
#define Sensor_Left_ID 12 
#define Sensor_Right_ID  11 
#define Sensor_Back_ID  13  
#define R 0.05 //радиус колес
#define L1 0.355 //расстояние между колесами по оси Y
#define L2 0.383// расстояние между колесами по оси X
#define cw 1
#define ccw -1
#define ACCEL_FACTOR                      0.000598550415   
#define GYRO_FACTOR                       0.0010642        
float wheel_separation = (L1+L2)/2;
int32_t goal_velocity[4] = {0,0,0,0};
uint8_t dxl_id[4] = {DXL_1, DXL_2, DXL_3, DXL_4};

DynamixelWorkbench dxl_wb;
cIMU IMU;

float theta_old = 0;
float theta = 0;
float delta_theta = 0;
static float orientation[4];

void init(){// функция инициализации dxl устройств
  IMU.begin();
  gyro_calib();
  
  const char *log;  
  bool result = false ;
  uint16_t model_number = 0;
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  for(int i = 0; i<4; i++){
    result = dxl_wb.ping(dxl_id[i], &model_number, &log); 
    
  }
  dxl_wb.ping(Sensor_Front_ID, &model_number, &log);
  dxl_wb.writeRegister((uint8_t)Sensor_Front_ID, "Registered", 0);
  
  dxl_wb.ping(Sensor_Left_ID, &model_number, &log);
  dxl_wb.ping(Sensor_Right_ID, &model_number, &log);
  dxl_wb.ping(Sensor_Back_ID, &model_number, &log);
  for(int i = 0; i<4; i++){
    dxl_wb.wheelMode(dxl_id[i], 0, &log);  
  }
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity");
}

void move_os(float vX,float vY,float wZ){//функция движения, принимает необходимую скорость по оси X, Y и вокруг оси Z
  const char *log;
  bool result = false;
  int64_t wheel_value[4] = {0, 0, 0, 0};
  double wheel_angular_velocity[4] = {0.0, 0.0, 0.0, 0.0};
  int32_t wheel_angular_velocity_real[4] = {0, 0, 0, 0};
  wheel_angular_velocity[0] = (1/R*(vX-vY+wheel_separation*(wZ))*cw);//расчитываем скорости для каждого из моторов с учетом направления его вращения
  wheel_angular_velocity[1] = (1/R*(vX+vY-wheel_separation*(wZ))*ccw);
  wheel_angular_velocity[2] = (1/R*(-vX-vY-wheel_separation*(wZ))*ccw);
  wheel_angular_velocity[3] = (1/R*(-vX+vY+wheel_separation*(wZ))*cw);
  for(int i = 0; i < 4; i++){
    wheel_angular_velocity_real[i] = (int32_t)(wheel_angular_velocity[i]*9.24/0.229);
//    result = dxl_wb.writeRegister((uint8_t)dxl_id[i], "Goal_Velocity", wheel_angular_velocity_real[i]);
  }
  result = dxl_wb.syncWrite((uint8_t)0, &wheel_angular_velocity_real[0], &log);
 // Serial.println(String(wheel_angular_velocity_real[0])+String(wheel_angular_velocity_real[1])+String(wheel_angular_velocity_real[2])+String(wheel_angular_velocity_real[3]));
//  / Serial.println(String(wheel_angular_velocity[0])+String(wheel_angular_velocity[1])+String(wheel_angular_velocity[2])+String(wheel_angular_velocity[3]));
}
void check_motors(){
  int32_t wheel_angular_velocity_real[4] = {0, 0, 0, 0};
  dxl_wb.syncWrite((uint8_t)0, &wheel_angular_velocity_real[0]);
}
void read_joint_state(float* current_velocity, float* current_effort){
  const char* log;
  for(int i = 0; i<4; i++){
    uint32_t get_data = 0;
    dxl_wb.readRegister(dxl_id[i], (uint16_t)128, (uint16_t)4, &get_data, &log);
    delay(1);
    current_velocity[i] = (int32_t)get_data*0.229/9.24;
    uint32_t get_data_eff = 0;
    dxl_wb.readRegister(dxl_id[i], (uint16_t)126, (uint16_t)2, &get_data_eff, &log);
    delay(1);
    if(abs(get_data_eff) > 1023)
      get_data_eff = 0;
    current_effort[i] = get_data_eff*0.001*294.2;
    if(dxl_id[i] == 2 || dxl_id[i] == 3)
      current_velocity[i] = -current_velocity[i];
      current_effort[i] = -current_effort[i];
 //   Serial.println("id: "+String(dxl_id[i])+" data: "+String(current_velocity[i]));
  }
}
float check_voltage(){
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);  
  float vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;
  return vol_value;
}
float* read_IMU(){
  static float imu_arr[10];
  IMU.update();
  imu_arr[0] = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_arr[1] = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_arr[2] = IMU.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_arr[3] = IMU.SEN.accADC[0] * ACCEL_FACTOR;
  imu_arr[4] = IMU.SEN.accADC[1] * ACCEL_FACTOR;
  imu_arr[5] = IMU.SEN.accADC[2] * ACCEL_FACTOR;
  imu_arr[6] = IMU.quat[0];
  imu_arr[7] = IMU.quat[1];
  imu_arr[8] = IMU.quat[2];
  imu_arr[9] = IMU.quat[3];
  return imu_arr;
}
void gyro_calib(){
   IMU.SEN.acc_cali_start();
   while( IMU.SEN.acc_cali_get_done() == false )
   {
      IMU.update();
   }
}
uint32_t* scan_front(){
  static uint32_t Frontarray[7];
  uint32_t respf;
  int idx;
  bool result = true;
  for(int i = 0; i < 7;i++){
    result = dxl_wb.readRegister((uint8_t)Sensor_Front_ID, (uint16_t)(24+2*i), (uint16_t)2, &respf);   
    if(result == false)
      Frontarray[i] = 0;
    else
      Frontarray[i] = respf;
   delay(1);

  }
  return Frontarray; 
}
uint32_t* scan_left(){
  static uint32_t Leftarray[7];
  uint32_t respl;
  int idx;
  bool result = true;
  for(int i = 0; i < 7;i++){
    result = dxl_wb.readRegister((uint8_t)Sensor_Left_ID, (uint16_t)(24+2*i), (uint16_t)2, &respl);   
    if(result == false)
      Leftarray[6-i] = 0;
    else
      Leftarray[6-i] = respl;
    delay(1);

  }
  return Leftarray;
}
uint32_t* scan_right(){
  static uint32_t Rightarray[7];
  uint32_t respr;
  int idx;
  bool result = true;
  for(int i = 0; i < 7;i++){
    result = dxl_wb.readRegister((uint8_t)Sensor_Right_ID, (uint16_t)(24+2*i), (uint16_t)2, &respr);   
    if(result == false)
      Rightarray[6-i] = 0;
    else
      Rightarray[6-i] = respr;
    delay(1);
  }
  return Rightarray;
}
uint32_t* scan_back(){
  static uint32_t Backarray[7];
  uint32_t respb;
  int idx;
  bool result = true;
  for(int i = 0; i < 7;i++){
    dxl_wb.readRegister((uint8_t)Sensor_Back_ID, (uint16_t)(24+2*i), (uint16_t)2, &respb);   
    if(result == false)
      Backarray[6-i] = 0;
    else
      Backarray[6-i] = respb;
    delay(1);
  }
  return Backarray;
}
