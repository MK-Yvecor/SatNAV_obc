#include "EX_Calibration_Values.h"
#include <math.h>
#include <Adafruit_MPU6050.h>

using namespace I2C_Sensors;

// INA219 class constructor
INA219::INA219(uint8_t addr, float ResitorValue, float MaxExpectedCurrent){
    INA_addr = addr;
    ResistorValue_ = ResitorValue;
    MaxExpectedCurrent_ = MaxExpectedCurrent;
}

//Initialization check
bool INA219::Init(){
    Wire.beginTransmission(INA_addr);
    uint8_t errCode = Wire.endTransmission();
    if(errCode == 0){
        return false;
    }
    else
    {
        return true;
    }    
}

bool INA219::ConfigureSensor(){

  Current_LSB = MaxExpectedCurrent_ / 32768.0f;
   uint16_t Calibration_Value = uint16_t(0.04096f /( Current_LSB * ResistorValue_));
   uint8_t u_ByteCalibrationValue = Calibration_Value >> 8;
   uint8_t l_ByteCalibrationValue = Calibration_Value & 0xFF;

   Wire.beginTransmission(INA_addr);
   Wire.write(0x05);
   Wire.write(u_ByteCalibrationValue);
   Wire.write(l_ByteCalibrationValue);

   uint8_t errCode = Wire.endTransmission();

    if(errCode == 0){
        return false;
    }

    else
    {
        return true;
    }
}

uint16_t INA219::readByte(uint8_t regAddress){

    uint16_t rawValue;
    uint8_t upperByte;
    uint8_t lowerByte;

    Wire.beginTransmission(INA_addr);
    Wire.write(regAddress);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)INA_addr ,(uint8_t)2);

    if(Wire.available()<2){
        return 0;
    }

    upperByte = Wire.read();
    lowerByte = Wire.read();

    rawValue = upperByte;
    rawValue = rawValue << 8;
    rawValue = rawValue | lowerByte;

    return rawValue;
}

/*
Calculation formulas are available in INA219 datasheet page 12 and 13: https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1760067833070&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219
All values after conversion are expressed in SI units: [V], [V], [A], [W].
*/

//result in V
float INA219::readShuntVoltage(){
    float result = (float)(int16_t)readByte(0x01) * 0.00001f;
    return result;
}

//Result in volts
float INA219::readBusVoltage(){
    float result = (float)(readByte(0x02) >> 3) * 0.004f;
    return result;
}

//result in W
float INA219::readPower(){
    float result = (float)readByte(0x03) * Current_LSB * 20.0f;
    return result;
}

//Result in A
float INA219::readCurrent(){
    double result = (float)(int16_t)readByte(0x04) * Current_LSB;
    return result;
}



/*

   EX_01 CLASS METHODS

*/

bool EX_01::ads_write_reg(uint8_t adr, uint8_t reg, uint8_t val, I2C_S_DRIVER *_I2C_S) {
    if(_I2C_S->WriteBegin(adr)){
    _I2C_S->I2C_STOP();
    return 1;
  }
  _I2C_S->Write(0x40 | (reg << 2), false);
  _I2C_S->Write(val, true);
  return 0;
}

bool EX_01::ads_start_conversion(uint8_t adr, I2C_S_DRIVER *_I2C_S) {
  _I2C_S->WriteBegin(adr);
  _I2C_S->Write(0x08, true);
  return 0;
}

bool EX_01::ads_reset() {
  if(I2C_S->WriteBegin(ex_adr)){
    I2C_S->I2C_STOP();
    return 1;
  }
  I2C_S->Write(0x06, true);
  delay(10);
  return 0;
}

bool EX_01::EX_Init() {
  ads_reset();
  ads_write_reg(ex_adr, 0x01, 0x00, I2C_S);
  return 0;
}

int16_t EX_01::ads_read_data_safe(uint8_t adr, I2C_S_DRIVER *_I2C_S) {
  _I2C_S->ReadBegin(adr, 0x10);
  uint8_t msb = _I2C_S->Read(false, true);
  uint8_t lsb = _I2C_S->Read(true, false);

  return (int16_t)((msb << 8) | lsb);
}

float EX_01::get_voltage(uint8_t adr, uint8_t mux_config, I2C_S_DRIVER *_I2C_S) {

  if(ads_write_reg(adr, 0x00, mux_config, _I2C_S)) { return 0; }

  ads_start_conversion(adr, _I2C_S);

  delay(60); 

  int16_t raw = ads_read_data_safe(adr, _I2C_S);

  return (raw * 2.048f) / 32768.0f;
}

void EX_01::read_EX_01_Data() {
  ex_raw_values.ex_red_channel = get_voltage(ex_adr, R_MUX, I2C_S);
  ex_raw_values.ex_green_channel = get_voltage(ex_adr, G_MUX, I2C_S);
  ex_raw_values.ex_blue_channel = get_voltage(ex_adr, B_MUX, I2C_S);
}

void EX_01::ex_signle_mux_series_covnersion(uint8_t *adr_arr, I2C_S_DRIVER *_I2C_S, uint8_t mux) {
  for(int i = 0; i<4; i++) {
    ads_write_reg(adr_arr[i], 0x00, mux, _I2C_S);
    ads_start_conversion(adr_arr[i], _I2C_S);
  }
  delay(60);
}

float EX_01::ex_read_data_from_single_mux(uint8_t adr,  I2C_S_DRIVER *_I2C_S) {

  int16_t raw = ads_read_data_safe(adr, _I2C_S);
  return (raw * 2.048f) / 32768.0f;

}

void EX_01::ex_set_calibration_values(const EX_Data * ex_cal_val_ptr, const I_exp * _exp_Ir, const I_exp * _exp_Ig, const I_exp * _exp_Ib) {
  ex_dark_current_calibration_values.ex_red_channel = ex_cal_val_ptr->ex_red_channel;
  ex_dark_current_calibration_values.ex_green_channel = ex_cal_val_ptr->ex_green_channel;
  ex_dark_current_calibration_values.ex_blue_channel = ex_cal_val_ptr->ex_blue_channel;

  exp_Ir.r_exp = _exp_Ir->r_exp;
  exp_Ir.g_exp = _exp_Ir->g_exp;
  exp_Ir.b_exp = _exp_Ir->b_exp;

  exp_Ig.r_exp = _exp_Ig->r_exp;
  exp_Ig.g_exp = _exp_Ig->g_exp;
  exp_Ig.b_exp = _exp_Ig->b_exp;

  exp_Ib.r_exp = _exp_Ib->r_exp;
  exp_Ib.g_exp = _exp_Ib->g_exp;
  exp_Ib.b_exp = _exp_Ib->b_exp;
}

void EX_01::ex_invert_exponents(EX_Data *ex_data_ptr) {

  ex_data_ptr->ex_red_channel = 2.048f - ex_data_ptr->ex_red_channel;
  ex_data_ptr->ex_green_channel = 2.048f - ex_data_ptr->ex_green_channel;
  ex_data_ptr->ex_blue_channel = 2.048f - ex_data_ptr->ex_blue_channel;

}

EX_01::EX_Data EX_01::get_raw_data(){
  return ex_raw_values;
}

void EX_01::set_raw_data(EX_01::EX_Data *_ex_data) {
  ex_raw_values.ex_red_channel = _ex_data->ex_red_channel;
  ex_raw_values.ex_green_channel = _ex_data->ex_green_channel;
  ex_raw_values.ex_blue_channel = _ex_data->ex_blue_channel;

}

void EX_01::ex_denoise(EX_Data *ex_data_ptr_read, EX_Data *ex_data_ptr_save) {

  ex_data_ptr_save->ex_red_channel = ex_data_ptr_read->ex_red_channel + (2.048f - ex_dark_current_calibration_values.ex_red_channel);
  ex_data_ptr_save->ex_green_channel = ex_data_ptr_read->ex_green_channel + (2.048f - ex_dark_current_calibration_values.ex_green_channel);
  ex_data_ptr_save->ex_blue_channel = ex_data_ptr_read->ex_blue_channel + (2.048f - ex_dark_current_calibration_values.ex_blue_channel);

}

void EX_01::ex_calc_final_data() {

  ex_denoise(&ex_raw_values, &ex_denoised);
  ex_invert_exponents(&ex_denoised);
 
  ex_data_final.ex_red_channel = (exp_Ir.r_exp * ex_denoised.ex_red_channel) + (exp_Ir.g_exp * ex_denoised.ex_green_channel) + (exp_Ir.b_exp * ex_denoised.ex_blue_channel);
  ex_data_final.ex_green_channel = (exp_Ig.r_exp * ex_denoised.ex_red_channel) + (exp_Ig.g_exp * ex_denoised.ex_green_channel) + (exp_Ig.b_exp * ex_denoised.ex_blue_channel);
  ex_data_final.ex_blue_channel = (exp_Ib.r_exp * ex_denoised.ex_red_channel) + (exp_Ib.g_exp * ex_denoised.ex_green_channel) + (exp_Ib.b_exp * ex_denoised.ex_blue_channel);

  if(ex_data_final.ex_red_channel < 0.000001f) {ex_data_final.ex_red_channel = 0.000001f;}
  if(ex_data_final.ex_red_channel > 2.048f) {ex_data_final.ex_red_channel = 2.048f;}
  if(ex_data_final.ex_green_channel < 0.000001f) {ex_data_final.ex_green_channel= 0.000001f;}
  if(ex_data_final.ex_green_channel > 2.048f) {ex_data_final.ex_green_channel = 2.048f;}
  if(ex_data_final.ex_blue_channel < 0.000001f) {ex_data_final.ex_blue_channel= 0.000001;}
  if(ex_data_final.ex_blue_channel > 2.048f) {ex_data_final.ex_blue_channel = 2.048f;}

}


float EX_01::Angstrom(EX_Data *ex_data_values_ptr) { 

  EX_Data NormalizedValues[4];
  EX_Data Tau[4];

  for(int i = 0; i<4; i++) {
    NormalizedValues[i].ex_red_channel = 0.0f;
    Tau[i].ex_red_channel = 0.0f;
    NormalizedValues[i].ex_green_channel = 0.0f;
    Tau[i].ex_green_channel = 0.0f;
    NormalizedValues[i].ex_blue_channel = 0.0f;
    Tau[i].ex_blue_channel = 0.0f;
  }

  for(int i = 0; i<4; i++) {

    NormalizedValues[i].ex_red_channel = ex_data_values_ptr[i].ex_red_channel / 2.048f;
    if(NormalizedValues[i].ex_red_channel < 0.000001f) {NormalizedValues[i].ex_red_channel = 0.000001f;}
    if(NormalizedValues[i].ex_red_channel > 1.0f) {NormalizedValues[i].ex_red_channel = 1.0f;}

    NormalizedValues[i].ex_green_channel = ex_data_values_ptr[i].ex_green_channel / 2.048f;
    if(NormalizedValues[i].ex_green_channel < 0.000001f) {NormalizedValues[i].ex_green_channel = 0.000001f;}
    if(NormalizedValues[i].ex_green_channel > 1.0f) {NormalizedValues[i].ex_green_channel = 1.0f;}

    NormalizedValues[i].ex_blue_channel = ex_data_values_ptr[i].ex_blue_channel / 2.048f;
    if(NormalizedValues[i].ex_blue_channel < 0.000001f) {NormalizedValues[i].ex_blue_channel = 0.000001f;}
    if(NormalizedValues[i].ex_blue_channel > 1.0f) {NormalizedValues[i].ex_blue_channel = 1.0f;}
  }

  for(int i = 0; i<4; i++) {
    Tau[i].ex_red_channel = -logf(NormalizedValues[i].ex_red_channel);
    Tau[i].ex_green_channel = -logf(NormalizedValues[i].ex_green_channel);
    Tau[i].ex_blue_channel = -logf(NormalizedValues[i].ex_blue_channel);
  }

  EX_Data Tau_mean = ex_mean(Tau);

  if(Tau_mean.ex_red_channel < 0.0001f) {Tau_mean.ex_red_channel = 0.0001f;}
  if(Tau_mean.ex_blue_channel < 0.0001f) {Tau_mean.ex_blue_channel = 0.0001f;}

  float alpha = -(logf((Tau_mean.ex_blue_channel / Tau_mean.ex_red_channel)) / logf((ex_lambda_blue / ex_lambda_red)));
  if(alpha < 0) { alpha = 0.0f; }         

  return alpha;

}

EX_01::EX_Data EX_01::ex_mean(EX_Data *ex_data_ptr) {

  EX_Data mean = {0.0f, 0.0f, 0.0f};

    for(int i = 0; i<4; i++) {
    mean.ex_red_channel += ex_data_ptr[i].ex_red_channel;
    mean.ex_green_channel += ex_data_ptr[i].ex_green_channel;
    mean.ex_blue_channel += ex_data_ptr[i].ex_blue_channel;
  }

  mean.ex_red_channel = mean.ex_red_channel / 4.0f;
  mean.ex_green_channel = mean.ex_green_channel / 4.0f;
  mean.ex_blue_channel = mean.ex_blue_channel / 4.0f;

  return mean;
}

float EX_01::ex_RGB_mean(EX_Data *ex_data_ptr) {
  float RGB_mean_single;

  RGB_mean_single = ex_data_ptr->ex_red_channel + ex_data_ptr->ex_green_channel + ex_data_ptr->ex_blue_channel;
  RGB_mean_single = RGB_mean_single / 3.0f;

  return RGB_mean_single;
}


int EX_01::ex_max_value(float *ex_normalized_values) {
  float max_value = ex_normalized_values[0];
  int ex_max_val_index = 0;

  for(int i = 1; i < 4; i++) {
    if(max_value < ex_normalized_values[i]) {
      ex_max_val_index = i;
    }
  }

  return ex_max_val_index;
}

float EX_01::ex_sun_azimuth_angle(float *ex_RGB_mean_val) {

  float angle;
  float deltaY = ex_RGB_mean_val[0] - ex_RGB_mean_val[2];
  float deltaX = ex_RGB_mean_val[1] - ex_RGB_mean_val[3];

  angle = atan2f(deltaY, deltaX) * (180.0f / float(M_PI));
  if(angle < 0) {
    angle = 360.0f + angle;
  }

  return angle;
}


float EX_01::ex_BR_ratio(float ex_blue_channel_val, float ex_red_channel_val) {
  float ex_br_ratio = ex_blue_channel_val / ex_red_channel_val;
  return ex_br_ratio;
}

EX_01::EX_Data EX_01::Get_EX_FinalValues() {
  return ex_data_final;
}

/*

  MPU 6050

*/

bool::MPU6050::ConfigureSensor() {
  if(!AF_MPU6050.begin(0x68)) {
    return false;
  }

  AF_MPU6050.setSampleRateDivisor(19); //20Hz
  AF_MPU6050.setAccelerometerRange(MPU6050_RANGE_16_G);
  AF_MPU6050.setGyroRange(MPU6050_RANGE_500_DEG);

  return true;

}

void::MPU6050::ReadDataIMU() {

  AF_MPU6050.getEvent(&a,&g,&t);

  readDataAccel.AX = a.acceleration.x;
  readDataAccel.AY = a.acceleration.y;
  readDataAccel.AZ = a.acceleration.z;

  readDataGyro.GX = g.gyro.x;
  readDataGyro.GY = g.gyro.y;
  readDataGyro.GZ = g.gyro.z;

}

MPU6050::GyroData MPU6050::GetGyroData(){
  return readDataGyro;
}

MPU6050::AccelData MPU6050::GetAccelData(){
  return readDataAccel;
}


void::MPU6050::CalibrateIMU(int samples) {

  MPU6050::AccelData _AccelOffset;
  MPU6050::GyroData _GyroOffset;

  AccelOffset.AX = 0;
  AccelOffset.AY = 0;
  AccelOffset.AZ = 0;

  GyroOffset.GX = 0;
  GyroOffset.GY = 0;
  GyroOffset.GZ = 0;

  

  for(int i = 0; i<samples; i++) {

      MPU6050::ReadDataIMU();
      _AccelOffset = MPU6050::GetAccelData();
      _GyroOffset = MPU6050::GetGyroData();

      AccelOffset.AX += _AccelOffset.AX;
      AccelOffset.AY += _AccelOffset.AY;
      AccelOffset.AZ += _AccelOffset.AZ;

      GyroOffset.GX += _GyroOffset.GX;
      GyroOffset.GY += _GyroOffset.GY;
      GyroOffset.GZ += _GyroOffset.GZ;

      delay(10);
  }

  AccelOffset.AX = AccelOffset.AX / samples;
  AccelOffset.AY = AccelOffset.AY / samples;
  AccelOffset.AZ = AccelOffset.AZ / samples;

  GyroOffset.GX = GyroOffset.GX / samples;
  GyroOffset.GY = GyroOffset.GY / samples;
  GyroOffset.GZ = GyroOffset.GZ / samples;

}