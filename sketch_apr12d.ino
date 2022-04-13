#include <CAN.h>
#include "ODriveTeensyCAN.h"

void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
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

ODriveTeensyCAN odrive(8, &send_cb, &recv_cb);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
