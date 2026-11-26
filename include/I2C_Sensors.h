#include <Arduino.h>
#include <Wire.h>
#include "I2C.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

//list of default addresses of I2C devices used in SatNAV. If the addresses are diffrent you can specify a custom address in class constructor

#define BMP280_ADDR 0x76
#define MPU6050_ADDR 0x68
#define INA219_ADDR 0x40

namespace I2C_Sensors{
    //Class for INA219 measurements
    class INA219 {

    public:
    // INA219 class constructor
    INA219(uint8_t addr_, float ResitorValue, float MaxExpectedCurrent);
    //Initialization check
    bool Init();
    //Writes all needed values to the configuration register
    bool ConfigureSensor();
    // Universal function to read 2 bytes of incoming data from the sensor
    uint16_t readByte(uint8_t regAddress);
    uint16_t writeByte(uint8_t regAddress);
    float readShuntVoltage();
    float readBusVoltage();
    float readCurrent();
    float readPower();

    private:

    uint8_t INA_addr;
    //Expressed in Ohms
    float ResistorValue_;
    //Expressed in Amps
    float MaxExpectedCurrent_;
    //Current LSB value
    float Current_LSB;
};





class EX_01 {

    public: 

    //ADC Address, SDA, SCL, Delay
    EX_01(uint8_t _ex_adr, I2C_S_DRIVER *I2C_S_ptr) {
        I2C_S = I2C_S_ptr;
        ex_adr = _ex_adr;
    }
    
    struct EX_Data {
        float ex_red_channel;
        float ex_green_channel;
        float ex_blue_channel;
    };
    
    struct I_exp {
        float r_exp;
        float g_exp;
        float b_exp;
    };
    
    #define R_MUX 0x80U
    #define G_MUX 0x90U
    #define B_MUX 0xA0U
    
    // Static Methods
    static constexpr float ex_lambda_red = 0.62f;
    static constexpr float ex_lambda_green = 0.55f;
    static constexpr float ex_lambda_blue = 0.47f;
    static void ex_signle_mux_series_covnersion(uint8_t *adr_arr, I2C_S_DRIVER *_I2C_S, uint8_t mux);
    static float ex_read_data_from_single_mux(uint8_t adr_arr, I2C_S_DRIVER *_I2C_S);
    static EX_Data ex_mean(EX_Data *ex_data_ptr);
    static float ex_RGB_mean(EX_Data *ex_data_ptr);
    static int ex_max_value(float *ex_normalized_values);
    static float ex_sun_azimuth_angle(float *ex_RGB_mean_val);
    static float Angstrom(EX_Data *ex_data_values_ptr);
    
    // Methods - individual for each sensor
    
    bool EX_Init();
    void read_EX_01_Data();
    EX_Data get_raw_data();
    void set_raw_data(EX_Data *_ex_data);
    
    void ex_set_calibration_values(const EX_Data * ex_cal_val_ptr, const I_exp * _exp_Ir, const I_exp * _exp_Ig, const I_exp * _exp_Ib);
    void ex_invert_exponents(EX_Data *ex_data_ptr);
    void ex_denoise(EX_Data *ex_data_ptr_read, EX_Data *ex_data_ptr_save);
    void ex_calc_final_data();
    
    EX_Data Get_EX_FinalValues();
    
    float ex_BR_ratio(float ex_blue_channel_val, float ex_red_channel_val);

    private:

    uint8_t ex_adr;
    
    I2C_S_DRIVER *I2C_S;

    I_exp exp_Ir;
    I_exp exp_Ig;
    I_exp exp_Ib;

    EX_Data ex_raw_values;
    EX_Data ex_dark_current_calibration_values;
    EX_Data ex_denoised;
    EX_Data ex_data_final;
    
    static bool ads_write_reg(uint8_t adr, uint8_t reg, uint8_t val, I2C_S_DRIVER *I2C_S);
    static float get_voltage(uint8_t adr, uint8_t mux_config, I2C_S_DRIVER *I2C_S);
    static bool ads_start_conversion(uint8_t adr, I2C_S_DRIVER *I2C_S);
    static int16_t ads_read_data_safe(uint8_t adr, I2C_S_DRIVER *I2C_S);

    bool ads_reset();
    bool i2c_write(uint8_t data);
    uint8_t i2c_read(bool send_ack);
};


class MPU6050 {

    public: 
    // Initializes MPU6050 sensor and sets its range
    bool ConfigureSensor();
    
    struct GyroData{
        float GX;
        float GY;
        float GZ;
    };
    
    struct AccelData {
        float AX;
        float AY;
        float AZ;  
    };
    
    // Reads Accel and Gyro data. To get data from gyro and/or accel you need to invoke GetAccelData() and/or GetGyroData() after calling this method.
    void ReadDataIMU();

    void CalibrateIMU(int samples_count);

    // Returns GyroData
    GyroData GetGyroData();
    GyroData GetGyroDataOffset();

    // Returns AccelData
    AccelData GetAccelData();
    AccelData GetAccelDataOffset();

    private:
    Adafruit_MPU6050 AF_MPU6050;
    sensors_event_t a,g,t;

    AccelData readDataAccel;
    GyroData readDataGyro;

    AccelData AccelOffset;
    GyroData GyroOffset;

};
}
