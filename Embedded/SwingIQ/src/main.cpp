#include <Arduino.h>
#include <Adafruit_LSM6DS3TRC.h> 
#include <ArduinoBLE.h>
#include <SPI.h>
#include <SD.h>
#include <queue>
#include <cppQueue.h>

#define SD_CS D6
#define BUTTON_PIN D0
#define LED_PIN D7

char bleName[] = "SwingIQ";

Adafruit_LSM6DS3TRC lsm6ds;
Adafruit_Sensor *lsm_accel, *lsm_gyro, *lsm_temp;
BLEService devService("A175281E-0080-49AD-8228-4332777DE600");
BLEStringCharacteristic statusCharacteristic("A3AA43CC-E0FD-4E9E-81C0-DCF57D2D06FF", BLERead | BLENotify, 32); // 32 character string characteristic
BLEByteCharacteristic useButtonCharacteristic("32D849DD-2626-43EA-AF67-4B175EC1D130", BLERead | BLEWrite);

typedef struct imu_measurement{
  int32_t time;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} imu_m_t;

#define NB_ITEMS 4096
imu_m_t			q_dat[NB_ITEMS];
cppQueue	q(sizeof(imu_m_t), NB_ITEMS, FIFO, false, q_dat, sizeof(q_dat));	// Instantiate queue with static queue data arguments

#define COUNT_BUF_SIZE 64
int count = 0;
std::string base_filename = "/s%d.csv";
char filename[COUNT_BUF_SIZE] = {0};

void connectHandler(BLEDevice central) {
    // central connected event handler
    // Serial.print("Connected event, central: ");
    // Serial.println(central.address());
  }
  
  // listen for BLE disconnect events:
  void disconnectHandler(BLEDevice central) {
    // central disconnected event handler
    // Serial.print("Disconnected event, central: ");
    // Serial.println(central.address());
  }
  
  // listen for characteristic subscribed events:
  void characteristicSubscribed(BLEDevice central, BLECharacteristic thisChar) {
    // central wrote new value to characteristic, update LED
    // Serial.print("Characteristic subscribed. UUID: ");
    // Serial.println(thisChar.uuid());
  }
  
  // listen for characteristic unsubscribed events:
  void characteristicUnsubscribed(BLEDevice central, BLECharacteristic thisChar) {
    // central wrote new value to characteristic, update LED
    // Serial.print("Characteristic unsubscribed. UUID: ");
    // Serial.println(thisChar.uuid());
  }
  
  // listen for characteristic updated events:
  void characteristicUpdated(BLEDevice central, BLECharacteristic thisChar) {
    // central wrote new value to characteristic, update LED
    // Serial.print("Characteristic updated. UUID: ");
    // Serial.print(thisChar.uuid());
    // Serial.print("   value: ");
    byte incoming = 0;
    thisChar.readValue(incoming);
    // Serial.println(incoming);
  }

void setup(void) {

    Serial.begin(115200);
    
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    if (!BLE.begin()) {
        Serial.println("starting BLE failed");
        digitalWrite(LED_PIN, HIGH);
        while (true);
    }
    BLE.setLocalName(bleName);
    Serial.println(bleName);
    BLE.setAdvertisedService(devService);
    devService.addCharacteristic(statusCharacteristic);
    devService.addCharacteristic(useButtonCharacteristic);
    BLE.addService(devService);

    statusCharacteristic.writeValue("BLE Ready");
    useButtonCharacteristic.writeValue(0);

    // assign event handlers for connected, disconnected to peripheral
    BLE.setEventHandler(BLEConnected, connectHandler);
    BLE.setEventHandler(BLEDisconnected, disconnectHandler);

    // assign event handlers for characteristics:
    useButtonCharacteristic.setEventHandler(BLEUpdated, characteristicUpdated);
    useButtonCharacteristic.setEventHandler(BLESubscribed, characteristicSubscribed);
    useButtonCharacteristic.setEventHandler(BLEUnsubscribed, characteristicUnsubscribed);

    statusCharacteristic.setEventHandler(BLESubscribed, characteristicSubscribed);
    statusCharacteristic.setEventHandler(BLEUnsubscribed, characteristicUnsubscribed);

    BLE.advertise();

    Serial.println("Giving Time to Setup Notifications");
    digitalWrite(LED_PIN, HIGH);
    unsigned long long waitTime = millis();
    while(waitTime + 20000 > millis()){
        BLE.poll();
    }
    digitalWrite(LED_PIN, LOW);

    Serial.println("Initializing Adafruit LSM6DS");
  
    if (!lsm6ds.begin_I2C()) {
      Serial.println("Failed to find LSM6DS chip");
      statusCharacteristic.setValue("IMU Fail");
      while(1){
        BLE.poll();
      }
    }
  
    Serial.println("LSM6DS Found!");
    statusCharacteristic.setValue("IMU Ready");
    BLE.poll();

    lsm_accel = lsm6ds.getAccelerometerSensor();
    lsm6ds.setAccelDataRate(  LSM6DS_RATE_416_HZ);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm_accel->printSensorDetails();
    float accel_sample_rate = lsm6ds.accelerationSampleRate();
    Serial.print("Sample Rate: ");
    Serial.println(accel_sample_rate);
  
    lsm_gyro = lsm6ds.getGyroSensor();
    lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);
    lsm_gyro->printSensorDetails();
    float gyro_sample_rate = lsm6ds.gyroscopeSampleRate();
    Serial.print("Sample Rate: ");
    Serial.println(gyro_sample_rate);

    Serial.println("Closing Serial");
    Serial.end();

    if (!SD.begin(SD_CS)) {
        statusCharacteristic.writeValue("SD Fail");
        while(1){
            BLE.poll();
        }
    }

    statusCharacteristic.writeValue("SD Success");
    File root = SD.open("/");
    while(true){
        File entry =  root.openNextFile();
        if(!entry){
            break;
        }
        count++;
        entry.close();
    }
    count = 100;
}

void loop(void){
    BLE.poll();
    if(digitalRead(BUTTON_PIN)){
        while(digitalRead(BUTTON_PIN)); // wait for button release
        delay(10);
        if (!SD.begin(SD_CS)){
            statusCharacteristic.writeValue("SD Fail");
            while(1){
                BLE.poll();
            }
        }
        statusCharacteristic.setValue("Logging to File");
        BLE.poll();
        snprintf(filename, COUNT_BUF_SIZE, base_filename.c_str(), count);
        File measurement_file = SD.open(filename, FILE_WRITE);
        measurement_file.println("time (ms),Ax (m/s^2),Ay,Az,Gx (rads/s),Gy,Gz");
        statusCharacteristic.setValue(filename);
        BLE.poll();
        digitalWrite(LED_PIN, HIGH);
        imu_m_t temp;
        while(!digitalRead(BUTTON_PIN)){
            sensors_event_t accel;
            sensors_event_t gyro;
            while(!(lsm6ds.accelerationGyroscopeAvailable()));
            lsm6ds.getEvent(&accel, &gyro, nullptr);
            imu_m_t measurement = {.time = accel.timestamp, .ax=accel.acceleration.x, .ay=accel.acceleration.z, .az=accel.acceleration.z, .gx=gyro.gyro.x, .gy=gyro.gyro.y, .gz=gyro.gyro.z};
            if(!q.push(&measurement)){
              for(int i = 0; i < 12; i++){
                q.pop(&temp);
                measurement_file.print(temp.time); 
                measurement_file.print(","); measurement_file.print(temp.ax);
                measurement_file.print(","); measurement_file.print(temp.ay);
                measurement_file.print(","); measurement_file.print(temp.az);
                measurement_file.print(","); measurement_file.print(temp.gx);
                measurement_file.print(","); measurement_file.print(temp.gy);
                measurement_file.print(","); measurement_file.print(temp.gz);
                measurement_file.println();
              }
              q.push(&measurement);
            }
        }
        while(q.pop(&temp)){
          measurement_file.print(temp.time); 
          measurement_file.print(","); measurement_file.print(temp.ax);
          measurement_file.print(","); measurement_file.print(temp.ay);
          measurement_file.print(","); measurement_file.print(temp.az);
          measurement_file.print(","); measurement_file.print(temp.gx);
          measurement_file.print(","); measurement_file.print(temp.gy);
          measurement_file.print(","); measurement_file.print(temp.gz);
          measurement_file.println();
        }
        measurement_file.close();
        count++;
        digitalWrite(LED_PIN, LOW);
        statusCharacteristic.setValue("Finished Logging");
        BLE.poll();
        statusCharacteristic.setValue(filename);
        BLE.poll();
        delay(10000);
    }
}