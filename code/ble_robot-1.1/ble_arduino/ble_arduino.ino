#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

// Distance TOF stuff
#define XSHUT 6

SFEVL53L1X distanceSensor;
SFEVL53L1X distanceSensor2;

// IMU stuff
ICM_20948_I2C myICM;
#define AD0_VAL 1

// Motor stuff
#define motor1_a 7
#define motor1_b 12

#define motor2_a 11
#define motor2_b 13


//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "3528303c-7773-4ff0-9bc8-d237afeb78ab"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

float distance1 = 0;

// Variables for PID
char pid_data[150][30];
float kp = 0;
float ki = 0;
float kd = 0;
int sample_counter = 0;
int data_counter = 0;
int setpoint = 340;
int prev_err = 0;

//////////// Global Variables ////////////

enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  ECHO,
  DANCE,
  SET_VEL,
  GET_TIME_MILLIS,
  GET_TEMP_5s,
  GET_TEMP_5s_RAPID,
  GET_TOF_5s,
  GET_ACC_5s,
  GET_IMU_5s_rapid,
  GET_IMU_ToF_5s,
  SET_PID,
  DRIVE_AT_WALL

};

void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }

  // Handle the command type accordingly
  uint32_t get_temp_counter = 0;
  uint32_t interval = 1000;
  uint32_t rapid_interval = 1000;
  static uint32_t nextTime;
  static uint32_t rapid_nextTime;
  uint32_t starttime;
  uint32_t endtime;

  // variables for IMU
  float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt = 0, pitch = 0, roll = 0, yaw = 0;
  float Xm = 0, Ym = 0, Zm = 0, x = 0, y = 0;
  unsigned long last_time = millis();
  double pitch_a_LPF[] = { 0, 0 };
  const int n = 1;

  // variables for KF
  int speed = 0;




  switch (cmd_type) {
    case PING:
      tx_estring_value.clear();
      tx_estring_value.append("PONG");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;

    case SEND_TWO_INTS:
      int int_a, int_b;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_a);
      if (!success)
        return;

      // Extract the next value from the command string as an integer
      success = robot_cmd.get_next_value(int_b);
      if (!success)
        return;

      Serial.print("Two Integers: ");
      Serial.print(int_a);
      Serial.print(", ");
      Serial.println(int_b);

      break;

    case SEND_THREE_FLOATS:
      /*
             * Your code goes here.
             */

      break;

    case ECHO:

      char char_arr[MAX_MSG_SIZE];

      // Extract the next value from the command string as a character array
      success = robot_cmd.get_next_value(char_arr);
      if (!success)
        return;

      Serial.print("Robot says ->: ");
      Serial.print(char_arr);

      break;
    case DANCE:
      Serial.println("Look Ma, I'm Dancin'!");

      break;

    case SET_VEL:

      break;

    case GET_TIME_MILLIS:

      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(millis());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;

    case GET_TEMP_5s:

      // Refrence for loop 1 time per second:
      // https://forum.arduino.cc/t/getting-a-loop-to-execute-in-a-defined-amount-of-time/919975/2


      while (get_temp_counter < 5) {


        // check if it's time
        if (millis() - nextTime >= interval) {
          // update next time
          nextTime += interval;

          // do your action here
          tx_estring_value.clear();

          // Append Time
          tx_estring_value.append("T:");
          tx_estring_value.append(millis());
          tx_estring_value.append("|");

          // Append Temp
          tx_estring_value.append("C:");
          tx_estring_value.append(getTempDegF());
          tx_estring_value.append("|");

          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
          get_temp_counter++;
        }
      }

      break;

    case GET_TEMP_5s_RAPID:


      starttime = millis();
      endtime = starttime;
      // check if it's time
      while ((endtime - starttime) <= 5000) {

        // do your action here
        tx_estring_value.clear();

        // Append Time
        tx_estring_value.append("T:");
        tx_estring_value.append(millis());
        tx_estring_value.append("|");

        // Append Temp
        tx_estring_value.append("C:");
        tx_estring_value.append(getTempDegF());
        tx_estring_value.append("|");

        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        endtime = millis();
      }


      break;

    case GET_TOF_5s:


      starttime = millis();
      endtime = starttime;
      // check if it's time
      while ((endtime - starttime) <= 5000) {

        // do your action here
        tx_estring_value.clear();

        // Append Time
        tx_estring_value.append("T:");
        tx_estring_value.append(millis());
        tx_estring_value.append("|");

        if (distanceSensor.checkForDataReady()) {
          int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor
          tx_estring_value.append("mm:");
          tx_estring_value.append(distance);
          tx_estring_value.append("|");
          distanceSensor.clearInterrupt();
        }
        if (distanceSensor2.checkForDataReady()) {
          int distance2 = distanceSensor2.getDistance();
          tx_estring_value.append("mm:");
          tx_estring_value.append(distance2);
          tx_estring_value.append("|");
          distanceSensor2.clearInterrupt();
        }

        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        endtime = millis();
      }


      break;

    case GET_ACC_5s:
      starttime = millis();
      endtime = starttime;
      // check if it's time
      while ((endtime - starttime) <= 5000) {

        // do your action here
        if (!myICM.dataReady()) {
          continue;
        }
          myICM.getAGMT();

          tx_estring_value.clear();

          // Append Time
          tx_estring_value.append("T:");
          tx_estring_value.append(millis());
          tx_estring_value.append("|");

          pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
          roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
          tx_estring_value.append("pitch:");
          tx_estring_value.append(pitch_a);
          tx_estring_value.append("|");
          tx_estring_value.append("roll:");
          tx_estring_value.append(roll_a);
          tx_estring_value.append("|");


          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
        
        endtime = millis();
      }


      break;
    
    case GET_IMU_5s_rapid:
      starttime = millis();
      endtime = starttime;
      // check if it's time
      while ((endtime - starttime) <= 5000) {

        // do your action here
        if (myICM.dataReady()) {
          myICM.getAGMT();

          tx_estring_value.clear();

          // Append Time
          tx_estring_value.append("T:");
          tx_estring_value.append(millis());
          tx_estring_value.append("|");

          pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
          dt = (micros() - last_time) / 1.;
          last_time = micros();
          pitch_g = pitch_g + myICM.gyrX() * dt;
          pitch = (pitch + myICM.gyrX() * dt) * 0.9 + pitch_a * 0.1;

          roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
          roll_g = roll_g + myICM.gyrY() * dt;
          roll = (roll + myICM.gyrY() * dt) * 0.9 + roll_a * 0.1;

          tx_estring_value.append("pitch:");
          tx_estring_value.append(pitch);
          tx_estring_value.append("|");
          tx_estring_value.append("roll:");
          tx_estring_value.append(roll);
          tx_estring_value.append("|");


          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          // Serial.print("Sent back: ");
          // Serial.println(tx_estring_value.c_str());
        }
        endtime = millis();
      }


      break;

    case GET_IMU_ToF_5s:
      starttime = millis();
      endtime = starttime;
      // check if it's time
      while ((endtime - starttime) <= 30000) {

        // do your action here
        if (myICM.dataReady()) {
          myICM.getAGMT();

          tx_estring_value.clear();

          // Append Time
          tx_estring_value.append("T:");
          tx_estring_value.append(millis());
          tx_estring_value.append("|");

          pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
          dt = (micros() - last_time) / 1.;
          last_time = micros();
          pitch_g = pitch_g + myICM.gyrX() * dt;
          pitch = (pitch + myICM.gyrX() * dt) * 0.9 + pitch_a * 0.1;

          roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
          roll_g = roll_g + myICM.gyrY() * dt;
          roll = (roll + myICM.gyrY() * dt) * 0.9 + roll_a * 0.1;

          tx_estring_value.append("pitch:");
          tx_estring_value.append(pitch);
          tx_estring_value.append("|");
          tx_estring_value.append("roll:");
          tx_estring_value.append(roll);
          tx_estring_value.append("|");

        if (distanceSensor.checkForDataReady()) {
          int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor
          tx_estring_value.append("d1:");
          tx_estring_value.append(distance);
          tx_estring_value.append("|");
          distanceSensor.clearInterrupt();
        }
        if (distanceSensor2.checkForDataReady()) {
          int distance2 = distanceSensor2.getDistance();
          tx_estring_value.append("d2:");
          tx_estring_value.append(distance2);
          tx_estring_value.append("|");
          distanceSensor2.clearInterrupt();
        }


          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          // Serial.print("Sent back: ");
          // Serial.println(tx_estring_value.c_str());
        }
        endtime = millis();
      }


      break;

    case SET_PID:
        tx_estring_value.clear();
        
        float setkp;
        float setkd;

        success = robot_cmd.get_next_value(setkp);
        if (!success){return;}
        kp = setkp;
        Serial.print("Kp set:");
        Serial.println(kp);

        starttime = millis();
        
        while(millis() < starttime + 7000) {
          if (distanceSensor.checkForDataReady()) {
            distance1 = distanceSensor.getDistance();
            Serial.println(distance1);
          }
          pid();
        }
        motor_control('S', 0);
      
        // Send Data
        for(int i = 0; i < data_counter; i++) {
          tx_estring_value.clear();          
          tx_estring_value.append(pid_data[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
          tx_estring_value.clear();
        }
        data_counter = 0;

        //clear data
        Serial.println("Clearing data");
      for(int i = 0; i < 150; i++){
        for(int j = 0; j < 30; j++){
          pid_data[i][j] = '\0';
        }
      }
        break;

    case DRIVE_AT_WALL:
        tx_estring_value.clear();

<<<<<<< HEAD
        
        speed = 255;
        motor_control('F', speed);
        starttime = millis();
        while(millis() < starttime + 5000) {
          if (distanceSensor.checkForDataReady()) {
            distance1 = distanceSensor.getDistance(); 
          if(sample_counter == 30) {
=======
        starttime = millis();
        speed = 90;
        motor_control('F', speed);
        while(millis() < starttime + 3000) {

          if (distanceSensor2.checkForDataReady()) {
            distance1 = distanceSensor2.getDistance(); 
          if(sample_counter == 35) {
>>>>>>> f4647a333e3de2834fff0df66a823e3547a48ade
            char time_buff[20];
            char speed_buff[20];
            char distance_buff[20];

            int time = millis();

            itoa(time, time_buff, 10);
            itoa(speed, speed_buff, 10);
            itoa(distance1, distance_buff, 10);

            strcat(pid_data[data_counter], "|T:");
            strcat(pid_data[data_counter], time_buff);
            strcat(pid_data[data_counter], "|S:");
            strcat(pid_data[data_counter], speed_buff);
            strcat(pid_data[data_counter], "|D:");
            strcat(pid_data[data_counter], distance_buff);  
            data_counter++;
            sample_counter = 0;
          }
          else sample_counter += 1;
          }

        }
        motor_control('S', 0);
      
        // Send Data
        for(int i = 0; i < data_counter; i++) {
          tx_estring_value.clear();          
          tx_estring_value.append(pid_data[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
          tx_estring_value.clear();
        }
        data_counter = 0;

        //clear data
        Serial.println("Clearing data");
        for(int i = 0; i < 150; i++){
          for(int j = 0; j < 30; j++){
            pid_data[i][j] = '\0';
        }
      }

        break;
    /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  BLE.begin();
  Wire.begin();
  Wire.setClock(400000);

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);

  /*
     * An example using the EString
     */
  // Clear the contents of the EString before using it
  tx_estring_value.clear();

  // Append the string literal "[->"
  tx_estring_value.append("[->");

  // Append the float value
  tx_estring_value.append(9.0);

  // Append the string literal "<-]"
  tx_estring_value.append("<-]");

  // Write the value to the characteristic
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, LOW);
  distanceSensor.setI2CAddress(0x32);
  digitalWrite(XSHUT, HIGH);

  if (distanceSensor.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  if (distanceSensor2.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensors online!");
  distanceSensor.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  distanceSensor.startRanging();  //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();



  // bool initialized = false;
  // while (!initialized) {
  //   myICM.begin(Wire, AD0_VAL);
  //   Serial.print(F("Initialization of the sensor returned: "));
  //   Serial.println(myICM.statusString());
  //   if (myICM.status != ICM_20948_Stat_Ok) {
  //     Serial.println("Trying again...");
  //     delay(500);
  //   } else {
  //     initialized = true;
  //   }
  // }

  pinMode(7, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
}

void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {

    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000) {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void read_data() {
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

int bound_speed(float speed){
  if(speed >= 60){
    return 60;
  }
  else if (speed >= 5 && speed <= 45){
    return 45;
  }
  else if (speed < 5){
    return 0;
  }
  else{
    return speed;
  }
}
void motor_control(char control, float speed){
  // Move forward
  if(control == 'B'){
    analogWrite(7,  speed);
    analogWrite(12, 0);
    analogWrite(11, speed*1.25);
    analogWrite(13, 0);

  }
  // Move backward
  else if(control == 'F'){
    analogWrite(7,  0);
    analogWrite(12, speed);
    analogWrite(11, 0);
    analogWrite(13, speed*1.25);
  }

  // Turn right
  else if(control == 'R'){
    analogWrite(7,  0);
    analogWrite(12, speed);
    analogWrite(11, speed*1.25);
    analogWrite(13, 0);
  }

  // Turn left
  else if(control == 'L'){
    analogWrite(7,  speed);
    analogWrite(12, 0);
    analogWrite(11, 0);
    analogWrite(13, speed*1.25);
  }

  // Stop
  else if(control == 'S'){
    analogWrite(7,  0);
    analogWrite(12, 0);
    analogWrite(11, 0);
    analogWrite(13, 0);
  }
}

void pid(){
  int err = distance1 - setpoint;
  int delta_err = err - prev_err;
  // int speed = kp*err + delta_err*kd;
  int speed = kp*err;

  if(speed > 0) {
    motor_control('F', bound_speed(speed));
  }
  else if(speed < 0) {
    motor_control('B', bound_speed(-1*speed));
  }
  else{
    motor_control('S', 0);
  }

  prev_err = err;

  if(sample_counter == 50) {
    char time_buff[20];
    char speed_buff[20];
    char distance_buff[20];

    int time = millis();

    itoa(time, time_buff, 10);
    itoa(speed, speed_buff, 10);
    itoa(distance1, distance_buff, 10);

    strcat(pid_data[data_counter], "|T:");
    strcat(pid_data[data_counter], time_buff);
    strcat(pid_data[data_counter], "|S:");
    strcat(pid_data[data_counter], speed_buff);
    strcat(pid_data[data_counter], "|D:");
    strcat(pid_data[data_counter], distance_buff);  
    data_counter++;
    sample_counter = 0;
  }
  else sample_counter += 1;
}

void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      // Send data
      write_data();

      // Read data
      read_data();
    }

    Serial.println("Disconnected");
  }
}