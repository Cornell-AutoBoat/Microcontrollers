/*
  This example shows how to gather RTCM data over I2C and push it to a serial monitor.
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS myGNSS;

// Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
uint32_t serverBytesSent = 0; // Just a running total
long lastReport_ms = 0;       // Time of last report of bytes sent
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
    Serial.begin(115200); // You may need to increase this for high navigation rates!
    while (!Serial)
        ; // Wait for user to open terminal
    Serial.println(F("SparkFun u-blox GPS correction source"));

    Wire.begin();

    // myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

    if (myGNSS.begin() == false) // Connect to the u-blox module using Wire port
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }

    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // UBX+RTCM3 is not a valid option so we enable all three.

    myGNSS.setNavigationFrequency(1); // Set output in Hz. RTCM rarely benefits from >1Hz.

    // Disable all NMEA sentences
    bool response = myGNSS.newCfgValset(VAL_LAYER_RAM); // Use cfgValset to disable individual NMEA messages
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GST_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 0);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C, 0);
    response &= myGNSS.sendCfgValset(); // Send the configuration VALSET

    if (response)
        Serial.println(F("NMEA messages were configured successfully"));
    else
        Serial.println(F("NMEA message configuration failed!"));

    // Enable necessary RTCM sentences
    response &= myGNSS.newCfgValset(VAL_LAYER_RAM);                            // Create a new Configuration Item VALSET message
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); // Enable message 1005 to output through I2C port, message every second
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
    response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // Enable message 1230 every 10 seconds
    response &= myGNSS.sendCfgValset();                                         // Send the VALSET

    if (response == true)
    {
        Serial.println(F("RTCM messages enabled"));
    }
    else
    {
        Serial.println(F("RTCM failed to enable. Are you sure you have an ZED-F9P?"));
        while (1)
            ; // Freeze
    }

    //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ in ECEF coordinates so...
    // Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
    // For more infomation see Example12_setStaticPosition
    // Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
    // will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
    // See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
    //
    // If you were setting up a full GNSS station, you would want to save these settings to Battery-Backed RAM.
    // Because setting an incorrect static position will disable the ability to get a lock, we will save to RAM layer only in this example - not RAM_BBR.
    // With high precision 0.1mm parts. False indicates ECEF (instead of LLH).
    response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10, false, VAL_LAYER_RAM);
    if (response == false)
    {
        Serial.println(F("Failed to enter static position. Freezing..."));
        while (1)
            ;
    }
    else
        Serial.println(F("Static position set"));

    // Alternatively to setting a static position, you could do a survey-in
    // but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
    // myGNSS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m

    Serial.println(F("Module configuration complete"));
}

void loop()
{
    if (Serial.available()) // A key is pressed
        // Once here, program will stay in beginServing()
        beginServing();

    Serial.println(F("Press any key to start serving"));

    delay(1000);
}

void beginServing()
{
    Serial.println("Begin transmitting to caster. Press any key to stop");
    delay(10); // Wait for any serial to arrive
    while (Serial.available())
        Serial.read(); // Flush

    lastReport_ms = millis();

    // This is the main sending loop. We scan for new ublox data but processRTCM() is where the data actually gets sent out.
    while (Serial.available() == 0)
    {
        while (1)
        {
            if (Serial.available()) // If there was a user input pressed
                break;

            myGNSS.checkUblox(); // See if new data is available. Process bytes as they come in.

            delay(10);

            // Report some statistics every 250
            if (millis() - lastReport_ms > 250)
            {
                lastReport_ms += 250;
                Serial.printf("Total sent: %d\n", serverBytesSent);
            }
        }
    }

    Serial.println("User pressed a key");
    Serial.println("Disconnecting...");
    // ntripCaster.stop();

    delay(10);
    while (Serial.available())
        Serial.read(); // Flush any endlines or carriage returns
}

// This function gets called from the SparkFun u-blox Arduino Library.
// As each RTCM byte comes in you can specify what to do with it
// Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
    Serial.println(incoming);
}