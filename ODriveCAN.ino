#include <CAN.h>
#include "ODriveCAN.h"
#include <AsyncDelay.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 2); return obj; }

AsyncDelay slow_loop;
AsyncDelay fast_loop;

ODriveCAN odrive(16, &send_cb, &recv_cb);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

float pos, vel;

void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
//  Serial.println(arbitration_id, HEX);
  CAN.beginPacket(arbitration_id, data_size, rtr);
  CAN.write(data, data_size);
  CAN.endPacket();
}

bool recv_cb(uint32_t arbitration_id, uint8_t *data, uint8_t *data_size) {
  int packetSize = CAN.parsePacket();
  int availableBytes = CAN.available();
  *data_size = availableBytes;
  if (availableBytes && CAN.packetId() == arbitration_id) {
    for (int i = 0; i < availableBytes; i++) {
      int r = CAN.read();
      if (r == -1) return false;
      data[i] = (uint8_t)r;
    }
    return true;
  }
  return false;
}


bool displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);

    return system > 0 && gyro >=3;
}

void setupBNO055() {
  adafruit_bno055_offsets_t calibData;

  calibData.accel_offset_x = -36;
  calibData.accel_offset_y = -120;
  calibData.accel_offset_z = -12;
  calibData.gyro_offset_x = 0;
  calibData.gyro_offset_y = -2;
  calibData.gyro_offset_z = 0;
  calibData.mag_offset_x = -866;
  calibData.mag_offset_y = 350;
  calibData.mag_offset_z = -146;
  calibData.accel_radius = 1000;
  calibData.mag_radius = 790;

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("Restoring Calibration data to the BNO055...");
  bno.setSensorOffsets(calibData);
  delay(500);
  bno.setExtCrystalUse(true); // Crystal must be configured AFTER loading calibration

  while(!displayCalStatus())
    delay(500); // wait for gyro
}

float realAngle, angleSpeed;

void readGyroscope() {
  sensors_event_t event;
  bno.getEvent(&event);
  realAngle = -event.orientation.z; // positive in front, z is actually x!
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  angleSpeed = event.gyro.x * 57.2958;  // radians to degrees
}

void IRAM_ATTR onReceive(int packetSize) {
  if (CAN.packetId() == 0x201) {
    uint32_t error, state;
    odrive.ReceiveHeartBeat(&error, &state);
//    Serial.println(state & 0xff, HEX);  // current state
    if (error) {
      Serial << "HEART ";
      Serial.print(error, HEX);
      Serial << " ";
      Serial.println(state, HEX);
    } else {
//      Serial << '.';
    }
    return;
  }

  if (CAN.packetId() == 0x209) {
      odrive.ReceivePosVel(&pos, &vel);
//      Serial << "@";
  }

  return; // !!!!!!!!!!!!!!!!!!!

  // received a packet
  Serial.print("Received ");

  if (CAN.packetExtended()) {
    Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

  Serial.print("packet with id 0x");
  Serial.print(CAN.packetId(), HEX);

  if (CAN.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(CAN.packetDlc());
  } else {
    Serial.print(" and length ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets
//    while (CAN.available()) {
//      Serial.print(CAN.read(), HEX);
//      Serial.print(' ');
//    }
//    Serial.println();
  }

  Serial.println();
}

float start_p = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  // put your setup code here, to run once:
  Serial.println("BEGIN");
  
  CAN.setPins(12, 13);
  // start the CAN bus at 1Mbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  setupBNO055();

  // register the receive callback
  CAN.onReceive(onReceive);

  odrive.ClearErrors();

  delay(1000);
  Serial.println("CLOSED LOOP");
  odrive.RunState(ODriveCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);

  start_p = pos;

  slow_loop.start(1000, AsyncDelay::MILLIS);
  fast_loop.start(5, AsyncDelay::MILLIS);
}

int loop_i = 0;
float p = 0, v = 0;

void loop() {
  if (fast_loop.isExpired()) {
    readGyroscope();
    if (loop_i < 200) {
      v += 0.3 * 0.005;
      p += v;
//      p = start_p + 2*sin(loop_i / 10.0);
      odrive.SetPosition(p);
    }
    else if (loop_i < 400) {
      odrive.SetPosition(0);
    }
    else {
      return;  // !!!!!!!!!!!
    }

//    Serial << p;
//    Serial << " " << pos << " " << vel;
    Serial << " " << v;
//    Serial << " " << odrive.GetIQMeasured();
    Serial << " " << realAngle << " " << angleSpeed / 10;
    Serial << "\n";
    loop_i++;

    fast_loop.repeat();
  }
/*
  if (slow_loop.isExpired()) {
    int mot_err = odrive.GetMotorError();
    if (mot_err) {
      Serial << "Motor ERR: ";
      Serial.print(mot_err, HEX);
    }
    int enc_err = odrive.GetEncoderError();
    if (enc_err) {
      Serial.print("\tEncoder ERR: ");
      Serial.print(enc_err, HEX);
    }
    if (mot_err || enc_err)
      Serial << "\n";

    slow_loop.repeat();
  }
*/
}
