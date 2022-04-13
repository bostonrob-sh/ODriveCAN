#include <CAN.h>
#include "ODriveCAN.h"

void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
  Serial.println(arbitration_id, HEX);
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

ODriveCAN odrive(8, &send_cb, &recv_cb);

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
  
  delay(1000);
  Serial.println("CLOSED LOOP");
  odrive.RunState(ODriveCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
