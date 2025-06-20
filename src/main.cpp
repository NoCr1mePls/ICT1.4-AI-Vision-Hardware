#include <Adafruit_GPS.h>
#include <Seeed_Arduino_SSCMA.h>
#include <HardwareSerial.h>
#include "TrashFound.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "config.h"

#define GPS_DEBUG false         // Set to 'true' for debugging NMEA sentences.
#define AI_DEBUG false         // Set to 'true' to enable SSCMA debug output.
#define TRASHFOUND_DEBUG false // Set to 'true' to enable debug output for trash found.
#define SENDTRASHFOUND true    // Set to 'true' to send trash found data to the server.

bool makeCall(trash trash);

Adafruit_GPS GPS(&Wire);
HardwareSerial atSerial(0);
SSCMA AI;
bool ledOn = false;

void setup()
{
    // Setup serial port for debugging
    Serial.begin(115200); // Set the baud rate for the serial port
    Serial.println("Initializing...");

    pinMode(LED_BUILTIN, OUTPUT);    // Set the built-in LED pin as output
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED initially

    // Setup the WiFi connection
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");

    // Setup the GPS
    Serial.print("Sensing satellites...");
    GPS.begin(0x10);                               // 0x10 is the I2C address
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // Only output RMC sentences
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
    while (!GPS.fix)
    {
        GPS.read();

        if (GPS.newNMEAreceived())
        {
            if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
                continue;                   // we can fail to parse a sentence in which case we should just wait for another
        }
        if (GPS_DEBUG)
        {
            Serial.print("Location: ");
            Serial.print(GPS.latitude_fixed / 10000000.0f);
            Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude_fixed / 10000000.0f);
            Serial.println(GPS.lon);
            Serial.print(" Fix:");
            Serial.println(GPS.fix);
        }
    }
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED when GPS fix is acquired
    Serial.println("\nGPS fix acquired!");
    Serial.println("Initializing SSCMA...");

    // Setup the AI serial port
    AI.begin(&atSerial);
    delay(1000);
    Serial.println("Done...\nSystems ready!");
}

void loop()
{
    // Read the data from the GPS
    GPS.read();
    if (GPS_DEBUG)
    {
        Serial.print("Location: ");
        Serial.print(GPS.latitude_fixed / 10000000.0f);
        Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude_fixed / 10000000.0f);
        Serial.println(GPS.lon);
        Serial.print(" Fix:");
        Serial.println(GPS.fix);
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
                    .confidence = (float)AI.boxes()[i].score / 100,
                    .latitude = GPS.latitude_fixed / 10000000.0f,
                    .longitude = GPS.longitude_fixed / 10000000.0f};

                if (SENDTRASHFOUND)
                    makeCall(found);
            }
            else
            {
                found = {
                    .type = AI.boxes()[i].target,
                    .confidence = (float)AI.boxes()[i].score / 100,
                    .latitude = 0.0f, // No GPS fix, set to 0
                    .longitude = 0.0f};
            }
            if (AI_DEBUG)
            {
                Serial.print("Found target: ");
                Serial.print(AI.boxes()[i].target);
                Serial.print(", Confidence: ");
                Serial.println((float)(AI.boxes()[i].score) / 100);
            }
            if (TRASHFOUND_DEBUG)
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
    }
    else
    {
        if (TRASHFOUND_DEBUG)
        {
            Serial.println("No changes detected.");
        }
    }

    // Toggle LED on if satellites are not found
    if (GPS.fix && ledOn)
    {
        digitalWrite(LED_BUILTIN, LOW); // Turn LED off when fix is acquired
        ledOn = false;
    }
    else if (!GPS.fix && !ledOn)
    {
        digitalWrite(LED_BUILTIN, HIGH); // Turn LED on when no fix
        ledOn = true;
    }
}

bool makeCall(trash trash)
{
    // Make an HTTP POST request to the server
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;
        http.begin(SERVER_URL);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("token", API_KEY);
        String jsonData = "{";
        jsonData += "\"litter_classification\": " + String(trash.type) + ",";
        jsonData += "\"confidence\": " + String(trash.confidence, 2) + ",";
        jsonData += "\"location_latitude\": " + String(trash.latitude, 7) + ",";
        jsonData += "\"location_longitude\": " + String(trash.longitude, 7);
        jsonData += "}";
        int responseCode = http.POST(jsonData);

        if (responseCode > 0)
        {
            Serial.println("Response: " + http.getString());
            return true; // Successfully sent data
        }
        else
        {
            Serial.println("POST failed. Error: " + String(responseCode));
            return false; // Failed to send data
        }

        http.end();
    }
    else
    {
        Serial.println("WiFi not connected. Cannot send data.");
        return false; // WiFi not connected
    }
}