#include <Wire.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

TinyGPSPlus gps;

float prevLat = 0.0;
float prevLon = 0.0;
unsigned long prevTime = 0;
float prevSpeed = 0.0;

float vibrationThreshold = 2.5;

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  int xAccRaw = analogRead(xPin);
  int yAccRaw = analogRead(yPin);
  int zAccRaw = analogRead(zPin);

  float xAcc = (xAccRaw - 512.0) * 0.0039;
  float yAcc = (yAccRaw - 512.0) * 0.0039;
  float zAcc = (zAccRaw - 512.0) * 0.0039;

  float accelerationMagnitude = sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);

  Serial.print("Acceleration Magnitude: ");
  Serial.println(accelerationMagnitude, 6);

  if (accelerationMagnitude > vibrationThreshold) {
    Serial.println("Accident Detected!!!!");
  } else if (accelerationMagnitude > 1.5) {
    Serial.println("Vibrations Detected!");
  } else {
    Serial.println("No Vibrations");
  }

  DynamicJsonDocument jsonDoc(300); 

  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid() && gps.hdop.isValid() && gps.hdop.hdop() < 2.0) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        float distance = haversine(prevLat, prevLon, latitude, longitude);

        unsigned long currentTime = millis();
        float deltaTime = (currentTime - prevTime) / 1000.0;

        float speed = distance / deltaTime;

        float acceleration = (speed - prevSpeed) / deltaTime;

        prevLat = latitude;
        prevLon = longitude;
        prevTime = currentTime;
        prevSpeed = speed;

        jsonDoc["acceleration"] = accelerationMagnitude;
        jsonDoc["vibration_status"] = (accelerationMagnitude > vibrationThreshold) ? "Accident Detected!!!!" : (accelerationMagnitude > 1.5) ? "Vibrations Detected!" : "No Vibrations";
        jsonDoc["latitude"] = latitude;
        jsonDoc["longitude"] = longitude;
        jsonDoc["speed"] = speed;
        jsonDoc["acceleration_gps"] = acceleration;
        jsonDoc["hdop"] = gps.hdop.hdop();

        serializeJson(jsonDoc, Serial);
        Serial.println(); 

        delay(1000);
      }
    }
  }
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);

  float a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2.0) * sin(dLon / 2.0);
  float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  float radius = 6371000.0;

  return radius * c;
}
