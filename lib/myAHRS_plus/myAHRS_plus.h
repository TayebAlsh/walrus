#ifndef __MYAHRS_PLUS_H_INCLUDED__
#define __MYAHRS_PLUS_H_INCLUDED__

#include <Wire.h>

//myAHRS_plus register map

enum {
    I2C_SLAVE_REG_WHO_AM_I = 0x01,
    I2C_SLAVE_REG_REV_ID_MAJOR,
    I2C_SLAVE_REG_REV_ID_MINOR,
    I2C_SLAVE_REG_STATUS,

    // RAW DATA
    I2C_SLAVE_REG_I_ACC_X_LOW = 0x10,
    I2C_SLAVE_REG_I_ACC_X_HIGH,
    I2C_SLAVE_REG_I_ACC_Y_LOW,
    I2C_SLAVE_REG_I_ACC_Y_HIGH,
    I2C_SLAVE_REG_I_ACC_Z_LOW,
    I2C_SLAVE_REG_I_ACC_Z_HIGH,
    I2C_SLAVE_REG_I_GYRO_X_LOW,
    I2C_SLAVE_REG_I_GYRO_X_HIGH,
    I2C_SLAVE_REG_I_GYRO_Y_LOW,
    I2C_SLAVE_REG_I_GYRO_Y_HIGH,
    I2C_SLAVE_REG_I_GYRO_Z_LOW,
    I2C_SLAVE_REG_I_GYRO_Z_HIGH,
    I2C_SLAVE_REG_I_MAGNET_X_LOW,
    I2C_SLAVE_REG_I_MAGNET_X_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Y_LOW,
    I2C_SLAVE_REG_I_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Z_LOW,
    I2C_SLAVE_REG_I_MAGNET_Z_HIGH,

    // COMPENSATED DATA
    I2C_SLAVE_REG_C_ACC_X_LOW,
    I2C_SLAVE_REG_C_ACC_X_HIGH,
    I2C_SLAVE_REG_C_ACC_Y_LOW,
    I2C_SLAVE_REG_C_ACC_Y_HIGH,
    I2C_SLAVE_REG_C_ACC_Z_LOW,
    I2C_SLAVE_REG_C_ACC_Z_HIGH,
    I2C_SLAVE_REG_C_GYRO_X_LOW,
    I2C_SLAVE_REG_C_GYRO_X_HIGH,
    I2C_SLAVE_REG_C_GYRO_Y_LOW,
    I2C_SLAVE_REG_C_GYRO_Y_HIGH,
    I2C_SLAVE_REG_C_GYRO_Z_LOW,
    I2C_SLAVE_REG_C_GYRO_Z_HIGH,
    I2C_SLAVE_REG_C_MAGNET_X_LOW,
    I2C_SLAVE_REG_C_MAGNET_X_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Y_LOW,
    I2C_SLAVE_REG_C_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Z_LOW,
    I2C_SLAVE_REG_C_MAGNET_Z_HIGH,
    I2C_SLAVE_REG_C_TEMPERATURE_LOW,
    I2C_SLAVE_REG_C_TEMPERATURE_HIGH,

    // Attitude - Euler angle
    I2C_SLAVE_REG_ROLL_LOW,
    I2C_SLAVE_REG_ROLL_HIGH,
    I2C_SLAVE_REG_PITCH_LOW,
    I2C_SLAVE_REG_PITCH_HIGH,
    I2C_SLAVE_REG_YAW_LOW,
    I2C_SLAVE_REG_YAW_HIGH,
    
    //Attitude - Quaternion
    I2C_SLAVE_REG_QUATERNIAN_X_LOW,
    I2C_SLAVE_REG_QUATERNIAN_X_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Y_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Y_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Z_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Z_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_W_LOW,
    I2C_SLAVE_REG_QUATERNIAN_W_HIGH,
};

bool readSensor(uint8_t reg_add, uint8_t* buff , uint8_t len)
{
    Wire1.beginTransmission((uint8_t)MYAHRS_I2C_ADDRESS);  
    Wire1.write(reg_add); 
    Wire1.endTransmission(false); 
    Wire1.requestFrom((uint8_t)MYAHRS_I2C_ADDRESS, len);

    uint8_t cnt = 0;
    while(Wire1.available()) {
        buff[cnt++] = Wire1.read();
    }

    return (cnt == len);
}

bool sensor_init()
{
  uint8_t buf_whomi[1];
  uint8_t buf_stat[1];
    
  if(readSensor(I2C_SLAVE_REG_WHO_AM_I, buf_whomi, 1) != 0xB1) {
    return false;
  }
  if(readSensor(I2C_SLAVE_REG_STATUS, buf_stat, 1) != 0x80) {
    return false;
  }
  return true;
}

bool write(uint8_t reg_add, uint8_t* buff , uint8_t len)
{
    Wire1.beginTransmission((uint8_t)MYAHRS_I2C_ADDRESS);
    
    Wire1.write(reg_add);                          

    for(uint8_t cnt=0; cnt<len; cnt++) {
        Wire1.write(buff[cnt]);                   
    }
    
    return Wire1.endTransmission(true) == 0;   
}

int read_raw_data()
{
  uint8_t buf_raw_data[18];
  
  readSensor(I2C_SLAVE_REG_I_ACC_X_LOW, buf_raw_data, 18);
  
  //Little endian
  int16_t acc_x = (buf_raw_data[1]<<8) | buf_raw_data[0];
  int16_t acc_y = (buf_raw_data[3]<<8) | buf_raw_data[2];
  int16_t acc_z = (buf_raw_data[5]<<8) | buf_raw_data[4];
  int16_t gyro_x = (buf_raw_data[7]<<8) | buf_raw_data[6];
  int16_t gyro_y = (buf_raw_data[9]<<8) | buf_raw_data[8];
  int16_t gyro_z = (buf_raw_data[11]<<8) | buf_raw_data[10];
  int16_t mag_x = (buf_raw_data[13]<<8) | buf_raw_data[12];
  int16_t mag_y = (buf_raw_data[15]<<8) | buf_raw_data[14];
  int16_t mag_z = (buf_raw_data[17]<<8) | buf_raw_data[16];

  Serial.print("#RAW ACCEL = ["); Serial.print(acc_x); Serial.print(", "); Serial.print(acc_y); Serial.print(", "); Serial.print(acc_z); Serial.print("]  ");
  Serial.print("#RAW GYRO = ["); Serial.print(gyro_x); Serial.print(", "); Serial.print(gyro_y); Serial.print(", "); Serial.print(gyro_z); Serial.print("]  ");
  Serial.print("#RAW MAGNET = ["); Serial.print(mag_x); Serial.print(", "); Serial.print(mag_y); Serial.print(", "); Serial.print(mag_z); Serial.print("]  ");
  Serial.println();
}

int read_compensated_data()
{
  uint8_t buf_comp_data[18];
  readSensor(I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);
  
  int16_t acc_c_x = (buf_comp_data[1]<<8) | buf_comp_data[0];
  int16_t acc_c_y = (buf_comp_data[3]<<8) | buf_comp_data[2];
  int16_t acc_c_z = (buf_comp_data[5]<<8) | buf_comp_data[4];
  int16_t gyro_c_x = (buf_comp_data[7]<<8) | buf_comp_data[6];
  int16_t gyro_c_y = (buf_comp_data[9]<<8) | buf_comp_data[8];
  int16_t gyro_c_z = (buf_comp_data[11]<<8) | buf_comp_data[10];
  int16_t mag_c_x = (buf_comp_data[13]<<8) | buf_comp_data[12];
  int16_t mag_c_y = (buf_comp_data[15]<<8) | buf_comp_data[14];
  int16_t mag_c_z = (buf_comp_data[17]<<8) | buf_comp_data[16];
  
  float comp_acc_x = (float)acc_c_x * 16.0 / 32767;
  float comp_acc_y = (float)acc_c_y * 16.0 / 32767;
  float comp_acc_z = (float)acc_c_z * 16.0 / 32767;
  float comp_gyro_x = (float)gyro_c_x * 2000 / 32767;
  float comp_gyro_y = (float)gyro_c_y * 2000 / 32767;
  float comp_gyro_z = (float)gyro_c_z * 2000 / 32767;
  float comp_mag_x = (float)mag_c_x * 0.3;
  float comp_mag_y = (float)mag_c_y * 0.3;
  float comp_mag_z = (float)mag_c_z * 0.3;

  Serial.print("#COMP ACCEL = ["); Serial.print(comp_acc_x); Serial.print(", "); Serial.print(comp_acc_y); Serial.print(", "); Serial.print(comp_acc_z); Serial.print("]  ");
  Serial.print("#COMP GYRO = ["); Serial.print(comp_gyro_x); Serial.print(", "); Serial.print(comp_gyro_y); Serial.print(", "); Serial.print(comp_gyro_z); Serial.print("]  ");
  Serial.print("#COMP MAGNET = ["); Serial.print(comp_mag_x); Serial.print(", "); Serial.print(comp_mag_y); Serial.print(", "); Serial.print(comp_mag_z); Serial.print("]  ");
  Serial.println();
}



// EULER ANGLE(RPY)
int read_euler()
{
  uint8_t buf_euler[6];
  
  readSensor(I2C_SLAVE_REG_ROLL_LOW, buf_euler, 6);
  
  int16_t euler_x = (buf_euler[1]<<8) | buf_euler[0];
  int16_t euler_y = (buf_euler[3]<<8) | buf_euler[2];
  int16_t euler_z = (buf_euler[5]<<8) | buf_euler[4];

  float roll = (float)euler_x * 180 / 32767;
  float pitch = (float)euler_y * 180 / 32767;
  float yaw = (float)euler_z * 180 / 32767;
  
  Serial.print("#EULER ANGLE = ["); Serial.print(roll); Serial.print(" , "); Serial.print(pitch); Serial.print(" , "); Serial.print(yaw); Serial.print("]  ");
  Serial.println();   Serial.println();
}

int read_quat()
{
  uint8_t buf_quat[8];
  
  readSensor(I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);
  
  int16_t quat_x = (buf_quat[1]<<8) | buf_quat[0];
  int16_t quat_y = (buf_quat[3]<<8) | buf_quat[2];
  int16_t quat_z = (buf_quat[5]<<8) | buf_quat[4];
  int16_t quat_w = (buf_quat[7]<<8) | buf_quat[6];
  
  float quaternion_x = (float)quat_x / 32767;
  float quaternion_y = (float)quat_y / 32767;
  float quaternion_z = (float)quat_z / 32767;
  float quaternion_w = (float)quat_w / 32767;
  
  Serial.print("#QUATERNION = ["); Serial.print(quaternion_x); Serial.print(" , "); Serial.print(quaternion_y); Serial.print(" , "); Serial.print(quaternion_z); Serial.print(" , "); Serial.print(quaternion_w); Serial.print("]  ");
  Serial.println();   Serial.println();
  
}


// READ REVISION ID
int read_rev_id()
{
  uint8_t id_1 = 0;
  uint8_t id_2 = 0;
  readSensor(I2C_SLAVE_REG_REV_ID_MAJOR, &id_1, 1);
  readSensor(I2C_SLAVE_REG_REV_ID_MAJOR, &id_2, 1);
  Serial.print("READ REVISION ID= "); Serial.print(id_1); Serial.println(id_2);
}

int who_am_i()
{
  uint8_t whomi = 0;
  readSensor(I2C_SLAVE_REG_WHO_AM_I, &whomi, 1);
  Serial.print("WHO AM I = "); Serial.println(whomi, HEX);
}

#endif