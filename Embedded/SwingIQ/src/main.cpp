#include <Arduino.h>
#include <Adafruit_LSM6DS3TRC.h> 
#include <SPI.h>
#include <SD.h>

#define SD_CS 7
#define BUTTON_PIN 0

Adafruit_LSM6DS3TRC lsm6ds;
Adafruit_Sensor *lsm_temp, *lsm_accel, *lsm_gyro;

#define COUNT_BUF_SIZE 64
int count = -1;
File measure_count_file;
std::string count_file_name = "/count.txt";
std::string base_filename = "/sample%d.csv";
char filename[COUNT_BUF_SIZE] = {0};
char count_file_buffer[COUNT_BUF_SIZE] = {0};

void setup(void) {
    Serial.begin(115200);
    while (!Serial)
      delay(10);
  
    Serial.println("Initializing Adafruit LSM6DS");
  
    if (!lsm6ds.begin_I2C()) {
      Serial.println("Failed to find LSM6DS chip");
      while (1) {
        delay(10);
      }
    }
  
    Serial.println("LSM6DS Found!");
    lsm_temp = lsm6ds.getTemperatureSensor();
    lsm_temp->printSensorDetails();
  
    lsm_accel = lsm6ds.getAccelerometerSensor();
    lsm_accel->printSensorDetails();
  
    lsm_gyro = lsm6ds.getGyroSensor();
    lsm_gyro->printSensorDetails();

    Serial.println("Initializing SD Card");
    if (!SD.begin(SD_CS)) {
        Serial.println("initialization failed!");
        while (1);
    }
    Serial.println("Initialization Complete.");

    pinMode(BUTTON_PIN, INPUT);
    if(SD.exists(count_file_name.c_str())){
        measure_count_file = SD.open("count.txt", FILE_READ);
        measure_count_file.read(count_file_buffer, COUNT_BUF_SIZE);
        measure_count_file.close();
        count = std::atoi(count_file_buffer);
    }else{
        measure_count_file = SD.open("count.txt", FILE_WRITE);
        measure_count_file.write(std::to_string(0).c_str());
        measure_count_file.close();
    }
    if (count < 0){
        Serial.println("Invalid Starting point for number in count.txt");
        while(1);
    }
}

void loop(void){
    Serial.println("Waiting to Start Logging");
    while(!digitalRead(BUTTON_PIN)); // wait for press
    while(digitalRead(BUTTON_PIN)); // wait for release
    snprintf(filename, COUNT_BUF_SIZE, base_filename.c_str(), count);
    File measurement_file = SD.open(filename, FILE_WRITE);
    measurement_file.println("time (ms),Ax (m/s^2),Ay,Az,Gx (rads/s),Gy,Gz,temperature (C)");
    // log until button is pressed again
    while(!digitalRead(BUTTON_PIN)){
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        lsm_temp->getEvent(&temp);
        lsm_accel->getEvent(&accel);
        lsm_gyro->getEvent(&gyro);
        measurement_file.print(millis()); 
        measurement_file.print(","); measurement_file.print(accel.acceleration.x);
        measurement_file.print(","); measurement_file.print(accel.acceleration.y);
        measurement_file.print(","); measurement_file.print(accel.acceleration.z);
        measurement_file.print(","); measurement_file.print(gyro.gyro.x);
        measurement_file.print(","); measurement_file.print(gyro.gyro.y);
        measurement_file.print(","); measurement_file.print(gyro.gyro.z);
        measurement_file.print(","); measurement_file.print(temp.temperature);
        measurement_file.println();
    }
    measurement_file.close();
    count++;
    SD.remove(count_file_name.c_str());
    measure_count_file = SD.open("count.txt", FILE_WRITE);
    measure_count_file.write(std::to_string(count).c_str());
    measure_count_file.close();

    // 1 second delay after recording
    delay(1000);
}