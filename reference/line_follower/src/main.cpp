#include <Arduino.h>

int sensor[5];
float Kp = 20.0;
float Ki = 0.0;
float sumError = 0.0;
float Kd = 0.0;
float previousError = 0.0;
void setup() {
    Serial.begin(115200);
    Serial.println("Robot is starting...");
}

void loop() {
    float position = 0;
    float positionSum = 0;
    int activeSensorCount = 0;
    sensor[0] = digitalRead(SENSOR_1);
    sensor[1] = digitalRead(SENSOR_2);
    sensor[2] = digitalRead(SENSOR_3);
    sensor[3] = digitalRead(SENSOR_4);
    sensor[4] = digitalRead(SENSOR_5);
    for (int i = 0; i < 5; i++) {
        if (sensor[i] == 1) {
            positionSum += i - 2;
            activeSensorCount++;
        }
    }
    if (activeSensorCount > 0) {
        position = positionSum / activeSensorCount;
    }
    else {
        Serial.println("LOST");
        return;
    }
    float error = position;
    sumError += error;
    float P = Kp * error;
    float I = Ki * sumError;
    float D = Kd * (error - previousError);
    previousError = error;
    float pid = P + I + D;
    Serial.print("PID: ");
    Serial.println(pid);
   

    Serial.print("Position: ");
    Serial.println(position);
    Serial.print("Error: ");
    Serial.println(error);
}