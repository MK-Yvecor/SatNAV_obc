#include "I2C.h"

void I2C_S_DRIVER::I2C_START() {

    GPIO.out_w1tc = (1 << SDA);
    GPIO.out_w1tc = (1 << SCL);

    GPIO.enable_w1tc = (1 << SCL);
    GPIO.enable_w1tc = (1 << SDA);

    esp_rom_delay_us(1);

    GPIO.enable_w1ts = (1 << SDA);
    esp_rom_delay_us(2);
    GPIO.enable_w1ts = (1 << SCL);
    esp_rom_delay_us(2);

}

void I2C_S_DRIVER::I2C_STOP() {

    GPIO.enable_w1ts = (1<<SDA);
    GPIO.enable_w1ts = (1<<SCL);

    esp_rom_delay_us(1);

    GPIO.enable_w1tc = (1<<SCL);
    esp_rom_delay_us(2);
    GPIO.enable_w1tc = (1<<SDA);
    esp_rom_delay_us(2);

}

uint8_t I2C_S_DRIVER::WriteBegin(uint8_t addr) {

    uint32_t ACK = 0;
    uint8_t targetADR = 0x00;
    targetADR = (addr<<1); 

    I2C_S_DRIVER::I2C_START();

    for(int i = 8; i>0; i--) {

        (targetADR >> (i-1)) & uint8_t(0x01) ? GPIO.enable_w1tc = (1 << SDA) : GPIO.enable_w1ts = (1 << SDA);
        esp_rom_delay_us(2);
        GPIO.enable_w1tc = (1 << SCL);
        esp_rom_delay_us(2);
        GPIO.enable_w1ts = (1 << SCL);

    }

    GPIO.enable_w1tc = (1 << SDA);

    esp_rom_delay_us(2);

    GPIO.enable_w1tc = (1 << SCL);

    esp_rom_delay_us(2);

    ACK = GPIO.in;
    if((ACK >> SDA) & (uint32_t)0x01)
        return 1;
    
    GPIO.enable_w1ts = (1 << SCL);
    // SDA in INPUT mode HiZ mode.

    esp_rom_delay_us(10);
    //GPIO.enable_w1ts = (1 << SDA);

    return 0;
}

uint8_t I2C_S_DRIVER::Write(uint8_t data, bool terminateTransmition) {

    for(int i = 8; i>0; i--) {

        (data >> (i-1)) & uint8_t(0x01) ? GPIO.enable_w1tc = (1 << SDA) : GPIO.enable_w1ts = (1 << SDA);
        esp_rom_delay_us(2);
        GPIO.enable_w1tc = (1 << SCL);
        esp_rom_delay_us(2);
        GPIO.enable_w1ts = (1 << SCL);

    }
    
    GPIO.enable_w1tc = (1 << SDA);

    esp_rom_delay_us(2);

    GPIO.enable_w1tc = (1 << SCL);

    esp_rom_delay_us(2);

    if((GPIO.in >> SDA) & (uint32_t)0x01)
        return 1;
    
    GPIO.enable_w1ts = (1 << SCL);
    // SDA in INPUT mode HiZ mode.

    esp_rom_delay_us(2);
    //GPIO.enable_w1ts = (1 << SDA);

    if(terminateTransmition) I2C_S_DRIVER::I2C_STOP();
    else return 0;
    
    return 0;
}

uint8_t I2C_S_DRIVER::ReadBegin(uint8_t addr, uint8_t data){

    uint8_t targetADR = 0x00;
    targetADR = (addr<<1) | 0x01; 
    WriteBegin(addr);
    Write(data, true);

    I2C_S_DRIVER::I2C_START();

    for(int i = 8; i>0; i--) {

        (targetADR >> (i-1)) & uint8_t(0x01) ? GPIO.enable_w1tc = (1 << SDA) : GPIO.enable_w1ts = (1 << SDA);
        esp_rom_delay_us(2);
        GPIO.enable_w1tc = (1 << SCL);
        esp_rom_delay_us(2);
        GPIO.enable_w1ts = (1 << SCL);

    }

    GPIO.enable_w1tc = (1 << SDA);

    esp_rom_delay_us(2);

    GPIO.enable_w1tc = (1 << SCL);

    esp_rom_delay_us(2);

    if((GPIO.in >> SDA) & (uint32_t)0x01)
        return 1;
    
    GPIO.enable_w1ts = (1 << SCL);
    // SDA in INPUT mode HiZ mode.

    esp_rom_delay_us(2);
    //GPIO.enable_w1ts = (1 << SDA);

    return 0;

}

uint8_t I2C_S_DRIVER::Read(bool terminateTransmition, bool sendACK) {

    uint8_t received = 0;

    GPIO.enable_w1tc = (1<<SDA);

    for(int i = 0; i<8; i++) {
        received = received<< 1;
        GPIO.enable_w1tc = (1 << SCL);
        esp_rom_delay_us(2);
        received |= (GPIO.in >> SDA) & 0x01 ? 0x01 : 0x00;
        esp_rom_delay_us(2);
        GPIO.enable_w1ts = (1 << SCL);
        esp_rom_delay_us(2);
        
    }

    if(sendACK){
        GPIO.enable_w1ts = (1<<SDA);
    }
    else{
        GPIO.enable_w1tc = (1<<SDA);
    }

    esp_rom_delay_us(2);

    GPIO.enable_w1tc = (1<<SCL);

    esp_rom_delay_us(5);

    GPIO.enable_w1ts = (1<<SCL);
    GPIO.enable_w1tc = (1<<SDA);

    if(terminateTransmition) I2C_S_DRIVER::I2C_STOP();
    else return received;

    return received;
}


uint8_t I2C_S_DRIVER::I2C_Scan(uint8_t ADDR) {


        if(WriteBegin(ADDR)==0)
        {
            I2C_S_DRIVER::I2C_STOP();
            return ADDR;
        }
        I2C_S_DRIVER::I2C_STOP();

    return 0;
}