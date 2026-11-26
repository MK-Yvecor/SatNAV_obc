#include <Arduino.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include "EX_Calibration_Values.h"
#include <Adafruit_BMP280.h>
#include <RadioLib.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include "GNSS.h"
#include "UART.h"
#include "Commands.h"

#define LED_PIN 12
#define LED_COUNT 1

#define L_CS 33
#define L_DIO0 37
#define L_RST 38

#define SCK 36
#define MISO 34
#define MOSI 35

const int rxPin = 13; // TX z L86
const int txPin = 14; // RX z L86

// I2C Pins
// SDA = 18, SCL = 17 

//Second I2C interface for EX01
// SDA = 2, SCL = 5
// EX_01 Address 0x40, 0x41, 0x44, 0x45

uint8_t EX_01_ADDR[4] = {0x40, 0x41, 0x44, 0x45};

using namespace I2C_Sensors;

SPIClass spiLoRa(FSPI);
I2C_S_DRIVER I2C_S(2,5);

INA219 INA219_(INA219_ADDR, 0.05f, 2.0f);
SX1278 radio = new Module(L_CS, L_DIO0, L_RST, -1, spiLoRa);
Adafruit_BMP280 _BMP280;

EX_01 EX_FRONT(0x40, &I2C_S);
EX_01 EX_RIGHT(0x41, &I2C_S);
EX_01 EX_BACK(0x44, &I2C_S);
EX_01 EX_LEFT(0x45, &I2C_S);

static EX_01 * const EX_SENSOR_LIST[] = {
  &EX_FRONT,
  &EX_RIGHT,
  &EX_BACK,
  &EX_LEFT
};

static const size_t EX_SENSOR_LIST_LENGTH = sizeof(EX_SENSOR_LIST) / sizeof(EX_01*);

MPU6050 MPU;
Adafruit_NeoPixel WS2812B(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
TinyGPSPlus gps;
GNSS _GNSS(&gps);

float PSRAM_SIZE;
float FLASH_SIZE;

float voltage;
float presssure;
float altitude;
float temperature;

double Longitude;
double Latitude;
double sats_num;

bool GNSS_NEW_FRAME = false;

unsigned long last_Read;

EX_01::EX_Data _EX_Data_RAW[4];
EX_01::EX_Data _EX_Data_Final[4];
float _EX_Data_RGB_mean[4];
EX_01::EX_Data _EX_Data_Final_mean;

float r_mux[4];
float g_mux[4];
float b_mux[4];

float Alpha;
float sun_angle;

MPU6050::GyroData GyroData;
MPU6050::AccelData AccelData;

bool framesent = false;

Commands comm;
UARTReader UR;
UARTSender US;

volatile bool transmitting = false;

byte buffer_LoRa[1024];
int BuffIndex = 0;

char c;

void OnTransmitDone(){
  transmitting = false;
}


void setup() {
  delay(500);

  Serial.begin(115200);
  Wire.begin(18,17);
  Wire.setClock(400000);
  WS2812B.begin();
  WS2812B.setPixelColor(0,0,10,0);
  WS2812B.show();
  delay(1000);
  
  spiLoRa.begin(SCK, MISO, MOSI, L_CS);

  //pinMode(2, INPUT_PULLUP);
  //pinMode(5, INPUT_PULLUP);

  Serial1.begin(9600, SERIAL_8N1, rxPin, txPin);

  US.AllocateMemoryForBuffer();
  
 if(!_BMP280.begin(0x76)){
    Serial.println("Failed to initialize BMP280");
    WS2812B.setPixelColor(50,0,0,0);
    WS2812B.show();
    delay(3000);
  }
  
  _BMP280.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X1,
    Adafruit_BMP280::SAMPLING_X1,
    Adafruit_BMP280::FILTER_OFF,
    Adafruit_BMP280::STANDBY_MS_1
  );
  
  if (!MPU.ConfigureSensor()) {   // 0x68 I2C Address
    Serial.println("MPU6050 not found!");
    WS2812B.setPixelColor(50,0,0,0);
    WS2812B.show();
    delay(3000);
  }
  
  if(INA219_.Init()){
    Serial.println("Failed to initialize INA219");
    WS2812B.setPixelColor(0,50,0,0);
    WS2812B.show();
    delay(3000);
  }
  
  if(INA219_.ConfigureSensor()){
      WS2812B.setPixelColor(0,50,0,0);
      WS2812B.show();
      Serial.println("Failed to configure INA219");
      delay(3000);
      //return;
  }


  //for(int i = 1; i<127; i++)
  //{
    //if(I2C_S.I2C_Scan(i) == i){
     // Serial.println(i, HEX);
    //}
 // 

  for(int i = 0; i<EX_SENSOR_LIST_LENGTH; i++) {
     EX_SENSOR_LIST[i]->EX_Init();
  }

  EX_FRONT.ex_set_calibration_values (
    &EX_Calibration_Values::_0x40_DarkCurrent,
     &EX_Calibration_Values::_0x40_IR,
      &EX_Calibration_Values::_0x40_IG,
       &EX_Calibration_Values::_0x40_IB
  );

  EX_RIGHT.ex_set_calibration_values( 
    &EX_Calibration_Values::_0x41_DarkCurrent,
     &EX_Calibration_Values::_0x41_IR, 
      &EX_Calibration_Values::_0x41_IG,
       &EX_Calibration_Values::_0x41_IB
  );

  EX_BACK.ex_set_calibration_values ( 
    &EX_Calibration_Values::_0x44_DarkCurrent,
      &EX_Calibration_Values::_0x44_IR,
        &EX_Calibration_Values::_0x44_IG,
          &EX_Calibration_Values::_0x44_IB
  );

  EX_LEFT.ex_set_calibration_values ( 
    &EX_Calibration_Values::_0x45_DarkCurrent,
     &EX_Calibration_Values::_0x45_IR,
      &EX_Calibration_Values::_0x45_IG,
       &EX_Calibration_Values::_0x45_IB
  );

  
  int state = radio.begin(433.96, 125.0, 9, 7, 0x12, 17);

  if(state != RADIOLIB_ERR_NONE){
    WS2812B.setPixelColor(0,50,0,0);
    WS2812B.show();
    Serial.print("Failed to initialize LoRa, ERR Code: ");
    Serial.println(state);
    delay(3000);
    return;
  }
  else { 
    Serial.println("Radio Ready"); 
    radio.setDio0Action(OnTransmitDone, RISING);
    for(int i = 0; i<=5; i++){
      delay(250);
      WS2812B.setPixelColor(0,50,50,0);
      WS2812B.show();
      delay(250);
      WS2812B.setPixelColor(0,0,0,0);
      WS2812B.show();
    }
  }
}

void wyswietlStatystyki() {
  Serial.print(F("Lokalizacja: ")); 
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Satelity: "));
  Serial.println(gps.satellites.value());

  WS2812B.setPixelColor(0,0,0,50);
  WS2812B.show();

}

void loop() {

  /*EX_LEFT.read_EX_01_Data();
  EX_LEFT.ex_calc_final_data();
  _EX_Data_Final[3] = EX_LEFT.Get_EX_FinalValues();
  _EX_Data_RAW[3] = EX_LEFT.get_raw_data();
  
  
  Serial.println("====== SENSOR: 3 ======");
  Serial.println("RAW");
  Serial.print("R: ");
  Serial.println(_EX_Data_RAW[3].ex_red_channel, 6);
  Serial.print("G: ");
  Serial.println(_EX_Data_RAW[3].ex_green_channel, 6);
  Serial.print("B: ");
  Serial.println(_EX_Data_RAW[3].ex_blue_channel, 6);
  
  Serial.println("Final");
  Serial.print("R: ");
  Serial.println(_EX_Data_Final[3].ex_red_channel, 6);
  Serial.print("G: ");
  Serial.println(_EX_Data_Final[3].ex_green_channel, 6);
  Serial.print("B: ");
  Serial.println(_EX_Data_Final[3].ex_blue_channel, 6);*/
  
  MPU.ReadDataIMU();
  AccelData = MPU.GetAccelData();
  GyroData = MPU.GetGyroData();

  voltage = INA219_.readBusVoltage();

  temperature = _BMP280.readTemperature();
  presssure = _BMP280.readPressure() / 100;
  altitude = _BMP280.readAltitude();

  EX_01::ex_signle_mux_series_covnersion(EX_01_ADDR, &I2C_S, R_MUX);
  for(int i = 0; i<4; i++){
    _EX_Data_RAW[i].ex_red_channel = EX_01::ex_read_data_from_single_mux(EX_01_ADDR[i], &I2C_S);
  }
    
  EX_01::ex_signle_mux_series_covnersion(EX_01_ADDR, &I2C_S, G_MUX);
  for(int i = 0; i<4; i++){
    _EX_Data_RAW[i].ex_green_channel = EX_01::ex_read_data_from_single_mux(EX_01_ADDR[i], &I2C_S);
  }
      
  EX_01::ex_signle_mux_series_covnersion(EX_01_ADDR, &I2C_S, B_MUX);
    for(int i = 0; i<4; i++){
      _EX_Data_RAW[i].ex_blue_channel = EX_01::ex_read_data_from_single_mux(EX_01_ADDR[i], &I2C_S);
  }

  if(_EX_Data_RAW[0].ex_red_channel != 0.0f || _EX_Data_RAW[1].ex_red_channel !=0.0f || _EX_Data_RAW[2].ex_red_channel !=0.0f || _EX_Data_RAW[3].ex_red_channel !=0.0f) 
  {   
                 
    for(int i = 0; i<4; i++){
      EX_SENSOR_LIST[i]->set_raw_data(&_EX_Data_RAW[i]);
      EX_SENSOR_LIST[i]->ex_calc_final_data();
      _EX_Data_Final[i] = EX_SENSOR_LIST[i]->Get_EX_FinalValues();
    }
            
            
    for(int i = 0; i<4; i++) {
      _EX_Data_RGB_mean[i] = EX_01::ex_RGB_mean(&_EX_Data_Final[i]);
    }
              
    _EX_Data_Final_mean = EX_01::ex_mean(_EX_Data_Final);
              
              
    sun_angle = EX_01::ex_sun_azimuth_angle(_EX_Data_RGB_mean);
    Alpha = EX_01::Angstrom(_EX_Data_Final);
              
              
    for(int i = 0; i<4; i++) {
      _EX_Data_RAW[i] = EX_SENSOR_LIST[i]->get_raw_data();
      /*Serial.print("R: ");
      Serial.println(_EX_Data_RAW[i].ex_red_channel, 6);
      Serial.print("G: ");
      Serial.println(_EX_Data_RAW[i].ex_green_channel, 6);
      Serial.print("B: ");
      Serial.println(_EX_Data_RAW[i].ex_blue_channel, 6);*/
                
    }
  }


  if(!transmitting) {
  /*US.WriteToBuffer(_EX_Data_Final[0].ex_red_channel, (char*)"FR");
  US.WriteToBuffer(_EX_Data_Final[0].ex_green_channel, (char*)"FG");
  US.WriteToBuffer(_EX_Data_Final[0].ex_blue_channel, (char*)"FB");

  US.WriteToBuffer(_EX_Data_Final[1].ex_red_channel, (char*)"RR");
  US.WriteToBuffer(_EX_Data_Final[1].ex_green_channel, (char*)"RG");
  US.WriteToBuffer(_EX_Data_Final[1].ex_blue_channel, (char*)"RB");

  US.WriteToBuffer(_EX_Data_Final[2].ex_red_channel, (char*)"BR");
  US.WriteToBuffer(_EX_Data_Final[2].ex_green_channel, (char*)"BG");
  US.WriteToBuffer(_EX_Data_Final[2].ex_blue_channel, (char*)"BB");

  US.WriteToBuffer(_EX_Data_Final[3].ex_red_channel, (char*)"LR");
  US.WriteToBuffer(_EX_Data_Final[3].ex_green_channel, (char*)"LG");
  US.WriteToBuffer(_EX_Data_Final[3].ex_blue_channel, (char*)"LB");*/

  US.WriteToBuffer(_EX_Data_Final_mean.ex_red_channel, (char*)"ER");
  US.WriteToBuffer(_EX_Data_Final_mean.ex_green_channel, (char*)"EG");
  US.WriteToBuffer(_EX_Data_Final_mean.ex_blue_channel, (char*)"EB");

  US.WriteToBuffer(altitude, (char*)"AL");
  US.WriteToBuffer(temperature, (char*)"TE");
  US.WriteToBuffer(presssure, (char*)"PR");
  US.WriteToBuffer(voltage, (char*)"VO");
  US.WriteToBuffer(GyroData.GX, (char*)"GX");
  US.WriteToBuffer(GyroData.GY, (char*)"GY");
  US.WriteToBuffer(GyroData.GZ, (char*)"GZ");
  US.WriteToBuffer(AccelData.AX, (char*)"AX");
  US.WriteToBuffer(AccelData.AY, (char*)"AY");
  US.WriteToBuffer(AccelData.AZ, (char*)"AZ");
  //US.WriteToBuffer(Alpha, (char*)"AL");

    if(US.GetBufferSize() > 0){
        memcpy(buffer_LoRa, US.GetBuffer(), US.GetBufferSize());
        int state = radio.startTransmit(buffer_LoRa, US.GetBufferSize());
        if(state == RADIOLIB_ERR_NONE){
          transmitting = true;
          Serial.println("TX Success");
        }
        else{
          Serial.print("Failed to TX, ERR CODE: ");
          Serial.println(state);
        }
    US.SendBuffer();
  }
}

delay(1000);

  /*for (int i = 0; i<4; i++) {
    Serial.print("====== SENSOR: ");
    Serial.print(i);
    Serial.println(" ======");
    Serial.print("R: ");
    Serial.println(_EX_Data_Final[i].ex_red_channel, 6);
    Serial.print("G: ");
    Serial.println(_EX_Data_Final[i].ex_green_channel, 6);
    Serial.print("B: ");
    Serial.println(_EX_Data_Final[i].ex_blue_channel, 6);
              
  }
                
  Serial.println("==== MEAN ====");
  Serial.print("R: ");
  Serial.println(_EX_Data_Final_mean.ex_red_channel, 6);
  Serial.print("G: ");
  Serial.println(_EX_Data_Final_mean.ex_green_channel, 6);
  Serial.print("B: ");
  Serial.println(_EX_Data_Final_mean.ex_blue_channel, 6);


  Serial.println("==== ANGSTREM ====");
  Serial.println(Alpha, 4);
  Serial.println("==== ANGLE ====");
  Serial.println(sun_angle, 4);

  Serial.println("");

  delay(100);*/

   /*
  Serial.print("R: ");
  Serial.println(_EX_Data.RED_Channel, 4);
  Serial.print("G: ");
  Serial.println(_EX_Data.GREEN_Channel, 4);
  Serial.print("B: ");
  Serial.println(_EX_Data.BLUE_Channel, 4);
  
  voltage = INA219_.readBusVoltage();
  Serial.println(voltage);

  
  temperature = _BMP280.readTemperature();
  presssure = _BMP280.readPressure() / 100;
  altitude = _BMP280.readAltitude();*/



  /*f(Serial1.available() > 0) {

    while(Serial1.available() > 0) {
      c = Serial1.read();
      if(_GNSS.NMEA_Encode(c))
      {
        //_GNSS.GetTime();
        Latitude = _GNSS.GetLatitude();
        Longitude = _GNSS.GetLongitude();
        sats_num = _GNSS.GetSats();
        GNSS_NEW_FRAME = true;
      }
    }

    
  }
  
  if(GNSS_NEW_FRAME) {
    Serial.print("LAT: ");
    Serial.println(Latitude, 6);
    Serial.print("LNG: ");
    Serial.println(Longitude, 6);
    Serial.print("SAT: ");
    Serial.println(sats_num);
    US.WriteToBuffer(Latitude, (char*)"LT");
    US.WriteToBuffer(Longitude, (char*)"LG");
    US.WriteToBuffer(sats_num, (char*)"ST");
    GNSS_NEW_FRAME = false;
  }
  else {
    if (millis() - last_Read > 1000)
    {
      last_Read = millis();
      Serial.println("-");
    }
  }


 */
}