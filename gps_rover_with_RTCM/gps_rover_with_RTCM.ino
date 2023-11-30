/*
Read RTCM data from a Serial port, push RTCM data to the GNSS module, and then pull the corrected position readings from the GNSS module.
*/

#include <SparkFun_u-blox_GNSS_v3.h> // http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> // friendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

// Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; // 5 RTCM messages take approximately ~300ms to arrive at 115200bps

// Added ****
double before_lat;
double before_long;
double after_lat;
double after_long;
//
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
    delay(1000);

    Serial.begin(115200);

    Wire.begin(); // Start I2C

    if (myGNSS.begin() == false) // Connect to the Ublox module using Wire port
    {
        Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }
    Serial.println(F("u-blox module connected"));

    myGNSS.setI2COutput(COM_TYPE_UBX);                                 // Turn off NMEA noise
    myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

    myGNSS.setNavigationFrequency(1); // Set output in Hz.

    while (Serial.available())
      Serial.read();
}

void loop()
{
    process_RTCM();

    delay(1000);
}

void process_RTCM()
{
    uint8_t rtcmData[512 * 4]; // Most incoming data is around 500 bytes but may be larger
    int rtcmCount = 0;
    int start_time = millis();

     // Get lat and long before correction
    before_lat = get_gps_lat();
    before_long = get_gps_long();

    Serial.print(F("Before correction (lat, long): "));
    Serial.print(before_lat);
    Serial.print(F(", "));
    Serial.println(before_long);
    while (Serial.available())
    {
        rtcmData[rtcmCount++] = Serial.read();
        if (rtcmCount == sizeof(rtcmData))
            break; ///// ******* NOTE: might want to break this loop after time period of reading.
        if (millis() - start_time > 5000)
            break;
    }

    if (rtcmCount > 0)
    {
        lastReceivedRTCM_ms = millis();

        // Push correction data to GNSS module
        myGNSS.pushRawData(rtcmData, rtcmCount, false);

        // Get corrected gps data (data after pushing rtcmdata to zed)
        after_lat = get_gps_lat();
        after_long = get_gps_long();

        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);
        Serial.print(F("Corrected (lat, long): "));
        Serial.print(after_lat);
        Serial.print(F(", "));
        Serial.println(after_long);
    }
}

// Returns high precision lat
double get_gps_lat()
{
    // Query module. The module only responds when a new position is available.
    if (myGNSS.getHPPOSLLH())
    {
        // First, let's collect the position data
        int32_t latitude = myGNSS.getHighResLatitude();
        int8_t latitudeHp = myGNSS.getHighResLatitudeHp();

        // Defines storage for the lat as double
        double d_lat;

        // Assemble the high precision latitude
        d_lat = ((double)latitude) / 10000000.0;      // Convert latitude from degrees * 10^-7 to degrees
        d_lat += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

        return d_lat;
    }
}

// Returns high precision longg
double get_gps_long()
{
    // Query module. The module only responds when a new position is available.
    if (myGNSS.getHPPOSLLH())
    {
        // First, let's collect the longitude position data
        int32_t longitude = myGNSS.getHighResLongitude();
        int8_t longitudeHp = myGNSS.getHighResLongitudeHp();

        // Defines storage for the long as double
        double d_long;

        // Assemble the high precision and longitude
        d_long = ((double)longitude) / 10000000.0;      // Convert longitude from degrees * 10^-7 to degrees
        d_long += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

        return d_long;
    }
}
