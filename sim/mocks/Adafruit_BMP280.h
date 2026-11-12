#pragma once
struct Adafruit_BMP280{ bool begin(int){return true;} float readPressure(); float readAltitude(float);};
