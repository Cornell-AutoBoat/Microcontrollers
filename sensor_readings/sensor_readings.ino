#include <OneWire.h> // DS18S20 temperature sensor

int leakPin = 2;     // Leak Signal Pin
int leak = 0;        // 0 = Dry , 1 = Leak
int DS18S20_Pin = 4; // DS18S20 Signal pin on digital 2
char tmpstring[10];

// Temperature chip i/o
OneWire ds(DS18S20_Pin); // on digital pin 2

void setup(void)
{
    Serial.begin(9600);

    pinMode(leakPin, INPUT);
}

void loop(void)
{
    Serial.println("BEGIN LOOP");

    // Temperature sensing
    // TODO: repeat for multiple temperature sensors.
    float temperature = getTemp();
    int tmp = (int)temperature;
    Serial.print("Temp: ");
    Serial.println(tmp);

    // Lead sensing
    // TODO: Repeat for multiple leak sensors
    leak = digitalRead(leakPin); // Read the Leak Sensor Pin

    Serial.print("Leak: ");
    Serial.println(leak);

    // Compass Data
    // TODO: Implement this following the sample code.

    Serial.println("END LOOP");
    delay(1000); // just here to slow down the output so it is easier to read
}

float getTemp()
{
    // Returns the temperature from one DS18S20 in Celsius

    byte data[12];
    byte addr[8];

    if (!ds.search(addr))
    {
        // no more sensors on chain, reset search
        ds.reset_search();
        return -1000;
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        Serial.println("CRC is not valid!");
        return -1000;
    }

    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
        Serial.print("Device is not recognized");
        return -1000;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (int i = 0; i < 9; i++)
    { // we need 9 bytes
        data[i] = ds.read();
    }

    ds.reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); // using two's compliment
    float TemperatureSum = tempRead / 16;

    return (TemperatureSum * 18 + 5) / 10 + 32;
}