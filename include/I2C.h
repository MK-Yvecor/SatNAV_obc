#include <stdint.h>
#include "Hal.h"
#include <soc\i2s_reg.h>
#include <soc\i2c_struct.h>
#include <driver\gpio.h>
#include <esp_rom_gpio.h>

class I2C_S_DRIVER {
    
    public:

    I2C_S_DRIVER(int _SDA, int _SCL){
        SDA = _SDA;
        SCL = _SCL;
    }

    uint8_t I2C_Scan(uint8_t ADDR);
    uint8_t WriteBegin(uint8_t addr);
    uint8_t Write(uint8_t data, bool terminateTransmition);
    uint8_t ReadBegin(uint8_t addr, uint8_t data);
    uint8_t Read(bool terminateTransmition, bool sendACK);
    void I2C_START();
    void I2C_STOP();

    private:

    int SDA, SCL;
};