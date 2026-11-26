#include <Arduino.h>
#include <TinyGPS++.h>

class GNSS {

    public:

    GNSS(TinyGPSPlus *__GNSS){
        _GNSS = __GNSS;
    }

    typedef struct
    {
        uint8_t Hour;
        uint8_t Minute;
        uint8_t Second;
        uint8_t Centisecond;

    } GNSS_Time;
    
    
    bool NMEA_Encode(char c);
    double GetLatitude();
    double GetLongitude();
    uint32_t GetSats();
    GNSS_Time GetTime();

    private:
    TinyGPSPlus *_GNSS;
    GNSS_Time _time;
    
    char c;

    bool IsLocUPD;
    bool IsTimUPD;
    bool IsSatUPD;


};