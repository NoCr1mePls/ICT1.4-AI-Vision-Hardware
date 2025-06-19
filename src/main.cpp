#include <Adafruit_GPS.h>
#include <Seeed_Arduino_SSCMA.h>
#include <HardwareSerial.h>
#include "TrashFound.h"

#define GPSECHO false          // Set to 'true' for debugging NMEA sentences.
#define AI_DEBUG false        // Set to 'true' to enable SSCMA debug output.
#define TrashFound_Debug true // Set to 'true' to enable debug output for trash found.

Adafruit_GPS GPS(&Wire);
HardwareSerial atSerial(0);
SSCMA AI;

void setup()
{
    // Setup serial port for debugging
    Serial.begin(115200);                    //Set the baud rate for the serial port
    Serial.println("Initializing...");       

    // Setup the GPS
    GPS.begin(0x10);                               // 0x10 is the I2C address
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // Only output RMC sentences
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate

    // Setup the AT serial port
    AI.begin(&atSerial);
    delay(1000);
    Serial.println("Done...");
}

void loop()
{
    // Read the data from the GPS
    GPS.read();
    if (GPSECHO)
    {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 8);
        Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 8);
        Serial.println(GPS.lon);
    }

    if (GPS.newNMEAreceived())
    {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return;                     // we can fail to parse a sentence in which case we should just wait for another
    }

    if (!AI.invoke(1, true))
    {
        trash found;
        for (int i = 0; i < AI.boxes().size(); i++)
        {
            if (GPS.fix)
            {
                found = {
                    .type = AI.boxes()[i].target,
                    .confidence = AI.boxes()[i].score,
                    .latitude = GPS.latitude,
                    .longitude = GPS.longitude};
            }
            else
            {
                found = {
                    .type = AI.boxes()[i].target,
                    .confidence = AI.boxes()[i].score,
                    .latitude = 0.0f,  // No GPS fix, set to 0
                    .longitude = 0.0f
                };
            }
            if (AI_DEBUG)
            {
                Serial.print("Found target: ");
                Serial.print(AI.boxes()[i].target);
                Serial.print(", Confidence: ");
                Serial.println(AI.boxes()[i].score);
            }
        }
        if (TrashFound_Debug)
        {
            Serial.print("Trash found: Type = ");
            Serial.print(found.type);
            Serial.print(", Confidence = ");
            Serial.print(found.confidence);
            Serial.print(", Latitude = ");
            Serial.print(found.latitude, 8);
            Serial.print(", Longitude = ");
            Serial.println(found.longitude, 8);
        }
    }
    else
    {
        Serial.println("No changes detected.");
    }
}