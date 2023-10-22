int pinTemp = A1; // This is where our Output data goes
void setup()
{
    // Begin serial communication at a baud rate of 9600:
    Serial.begin(9600);
}
void loop()
{
    // Get a reading from the temperature sensor:
    int reading = analogRead(pinTemp);
    // Convert the reading into voltage:
    float voltage = reading * (5000 / 1024.0);
    // Convert the voltage into the temperature in Celsius:
    float temperatureC = (voltage - 500) / 10;
    // Print the temperature in the Serial Monitor:
    Serial.print("Temperature is: ");
    Serial.print(temperatureC);
    Serial.print(" \xC2\xB0"); // shows degree symbol
    Serial.print("C, ");
    int TempF = temperatureC * 1.8 + 32;
    Serial.print(TempF);
    Serial.print(" \xC2\xB0"); // shows degree symbol
    Serial.println("F");
    delay(1000); // wait a second between readings
}