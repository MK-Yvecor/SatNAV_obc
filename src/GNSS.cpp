#include <Arduino.h>
#include "GNSS.h"


bool GNSS::NMEA_Encode(char c) {

    if(_GNSS->encode(c)){

        if(_GNSS->time.isUpdated()) IsTimUPD = true;
        if(_GNSS->location.isUpdated()) IsLocUPD = true;
        if(_GNSS->satellites.isUpdated()) IsSatUPD = true;

        if(IsTimUPD && IsLocUPD && IsSatUPD) {
            IsTimUPD = false;
            IsLocUPD = false;
            IsSatUPD = false;

            return 1;
        }

        return 0;
    }

    return 0;
}

double GNSS::GetLatitude(){
    return _GNSS->location.lat();
}

double GNSS::GetLongitude(){
    return _GNSS->location.lng();
}

uint32_t GNSS::GetSats() {
    return _GNSS->satellites.value();
}

GNSS::GNSS_Time GNSS::GetTime() {

    _time.Hour = _GNSS->time.hour();
    _time.Minute = _GNSS->time.minute();
    _time.Second = _GNSS->time.second();
    _time.Centisecond = _GNSS->time.centisecond();

    return _time;

}