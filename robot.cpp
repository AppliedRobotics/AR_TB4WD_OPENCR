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
  
//  Serial/.begin(9600);
  uint16_t model_number = 0;
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  
  
  for(int i = 0; i<4; i++){
    result = dxl_wb.ping(dxl_id[i], &model_number, &log); 
    
  }
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
  wheel_angular_velocity[0] = (1/R*(vX+vY+wheel_separation*(wZ))*cw);//расчитываем скорости для каждого из моторов с учетом направления его вращения
  wheel_angular_velocity[1] = (1/R*(vX+vY-wheel_separation*(wZ))*ccw);
  wheel_angular_velocity[2] = (1/R*(vX-vY-wheel_separation*(wZ))*ccw);
  wheel_angular_velocity[3] = (1/R*(vX-vY+wheel_separation*(wZ))*cw);
  for(int i = 0; i < 4; i++){
    wheel_angular_velocity_real[i] = (int32_t)(wheel_angular_velocity[i]*9.24/0.229);
  }
  result = dxl_wb.syncWrite((uint8_t)0, &wheel_angular_velocity_real[0], &log);
 // Serial.println(String(wheel_angular_velocity_real[0])+String(wheel_angular_velocity_real[1])+String(wheel_angular_velocity_real[2])+String(wheel_angular_velocity_real[3]));
//  / Serial.println(String(wheel_angular_velocity[0])+String(wheel_angular_velocity[1])+String(wheel_angular_velocity[2])+String(wheel_angular_velocity[3]));
}
void check_motors(){
  int32_t wheel_angular_velocity_real[4] = {0, 0, 0, 0};
  dxl_wb.syncWrite((uint8_t)0, &wheel_angular_velocity_real[0]);
}
void read_joint_state(float* current_velocity){
  const char* log;
  for(int i = 0; i<4; i++){
    uint32_t get_data = 0;
    dxl_wb.readRegister(dxl_id[i], (uint16_t)128, (uint16_t)4, &get_data, &log);
    current_velocity[i] = (int32_t)get_data*0.229/9.24;
    if(dxl_id[i] == 2 || dxl_id[i] == 3)
      current_velocity[i] = -current_velocity[i];
 //   Serial.println("id: "+String(dxl_id[i])+" data: "+String(current_velocity[i]));
  }
}
float check_voltage(){
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);  
  float vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;
  return vol_value;
}
sensor_msgs::Imu read_IMU(){
  sensor_msgs::Imu imu_msg_;
  IMU.update();
  imu_msg_.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg_.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg_.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  imu_msg_.orientation.w = IMU.quat[0];
  imu_msg_.orientation.x = IMU.quat[1];
  imu_msg_.orientation.y = IMU.quat[2];
  imu_msg_.orientation.z = IMU.quat[3];

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;
}
void gyro_calib(){
   IMU.SEN.acc_cali_start();
   while( IMU.SEN.acc_cali_get_done() == false )
   {
      IMU.update();
   }
}
void scan1(uint16_t* Frontarray, uint16_t* Leftarray, uint16_t* Rightarray, uint16_t* Backarray) // функция опроса датчиков
{
 // sendval();
//////////////////////  передний датчик //////////////////////
   uint32_t respf[14];
   int idx;
    dxl_wb.readRegister(Sensor_Front_ID, 24, 14, respf);
    
    idx = 0;
    Frontarray[0] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[1] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[2] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[3] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[4] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[5] = respf[idx] + (respf[idx + 1] << 8);
    idx += 2;
    Frontarray[6] = respf[idx] + (respf[idx + 1] << 8);
      
//////////////////////  левый датчик //////////////////////
    uint32_t respl[14];

    dxl_wb.readRegister(Sensor_Left_ID, 24, 14, respl);
      
    idx = 0;
    Leftarray[6] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[5] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[4] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[3] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[2] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[1] = respl[idx] + (respl[idx + 1] << 8);
    idx += 2;
    Leftarray[0] = respl[idx] + (respl[idx + 1] << 8);
    idx = 0;
      
//////////////////////  правый датчик //////////////////////
     uint32_t respr[14];

    dxl_wb.readRegister(Sensor_Right_ID, 24, 14, respr);
      
    idx = 0;
    Rightarray[6] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[5] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[4] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[3] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[2] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[1] = respr[idx] + (respr[idx + 1] << 8);
    idx += 2;
    Rightarray[0] = respr[idx] + (respr[idx + 1] << 8);
    idx = 0;
      
//////////////////////  задний датчик //////////////////////
    uint32_t respb[14];

    dxl_wb.readRegister(Sensor_Back_ID, 24, 14, respb);
      
    idx = 0;
    Backarray[6] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[5] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[4] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[3] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[2] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[1] = respb[idx] + (respb[idx + 1] << 8);
    idx += 2;
    Backarray[0] = respb[idx] + (respb[idx + 1] << 8);
    idx = 0;
      
} 
