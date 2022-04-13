#include "Arduino.h"
#include "ODriveCAN.h"
#include <CAN.h>

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

static const int NodeIDLength = 6;
static const int CommandIDLength = 5;

static const float feedforwardFactor = 1 / 0.001;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

void ODriveCAN::sendMessage(int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes) {
//    CAN_message_t return_msg;

    int arbitration_id = (axis_id << CommandIDLength) + cmd_id;
    if (!remote_transmission_request) {
        return send_cb(arbitration_id, signal_bytes, length, remote_transmission_request);
    }

    send_cb(arbitration_id, signal_bytes, length, remote_transmission_request);
    while (true) {
        if (recv_cb(arbitration_id, _data, &_data_size)) {
            memcpy(signal_bytes, _data, _data_size);
            return;
        }
    }
}

int ODriveCAN::Heartbeat() {
//    CAN_message_t return_msg;
//	if(Can0.read(return_msg) == 1) {
//		return (int)(return_msg.id >> 5);
//	} else {
//		return -1;
//	}
}

void ODriveCAN::SetPosition(float position) {
    SetPosition(position, 0.0f, 0.0f);
}

void ODriveCAN::SetPosition(float position, float velocity_feedforward) {
    SetPosition(position, velocity_feedforward, 0.0f);
}

void ODriveCAN::SetPosition(float position, float velocity_feedforward, float current_feedforward) {
    int16_t vel_ff = (int16_t) (feedforwardFactor * velocity_feedforward);
    int16_t curr_ff = (int16_t) (feedforwardFactor * current_feedforward);

    byte* position_b = (byte*) &position;
    byte* velocity_feedforward_b = (byte*) &vel_ff;
    byte* current_feedforward_b = (byte*) &curr_ff;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = position_b[0];
    msg_data[1] = position_b[1];
    msg_data[2] = position_b[2];
    msg_data[3] = position_b[3];
    msg_data[4] = velocity_feedforward_b[0];
    msg_data[5] = velocity_feedforward_b[1];
    msg_data[6] = current_feedforward_b[0];
    msg_data[7] = current_feedforward_b[1];

    sendMessage(CMD_ID_SET_INPUT_POS, false, 8, position_b);
}

void ODriveCAN::SetVelocity(float velocity) {
    SetVelocity(velocity, 0.0f);
}

void ODriveCAN::SetVelocity(float velocity, float current_feedforward) {
    byte* velocity_b = (byte*) &velocity;
    byte* current_feedforward_b = (byte*) &current_feedforward;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_b[0];
    msg_data[1] = velocity_b[1];
    msg_data[2] = velocity_b[2];
    msg_data[3] = velocity_b[3];
    msg_data[4] = current_feedforward_b[0];
    msg_data[5] = current_feedforward_b[1];
    msg_data[6] = current_feedforward_b[2];
    msg_data[7] = current_feedforward_b[3];
    
    sendMessage(CMD_ID_SET_INPUT_VEL, false, 8, velocity_b);
}

void ODriveCAN::SetVelocityLimit(float velocity_limit) {
    byte* velocity_limit_b = (byte*) &velocity_limit;

    sendMessage(CMD_ID_SET_VELOCITY_LIMIT, false, 4, velocity_limit_b);
}

void ODriveCAN::SetTorque(float torque) {
    byte* torque_b = (byte*) &torque;

    sendMessage(CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
}

void ODriveCAN::ClearErrors() {
    sendMessage(CMD_ID_CLEAR_ERRORS, false, 0, 0);
}

float ODriveCAN::GetPosition() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

float ODriveCAN::GetVelocity() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[4];
    *((uint8_t *)(&output) + 1) = msg_data[5];
    *((uint8_t *)(&output) + 2) = msg_data[6];
    *((uint8_t *)(&output) + 3) = msg_data[7];
    return output;
}

uint32_t ODriveCAN::GetMotorError() {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(CMD_ID_GET_MOTOR_ERROR, true, 4, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

uint32_t ODriveCAN::GetEncoderError() {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(CMD_ID_GET_ENCODER_ERROR, true, 4, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

uint32_t ODriveCAN::GetAxisError() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

//    CAN_message_t return_msg;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

//    while (true) {
//        if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
//            memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
//            *((uint8_t *)(&output) + 0) = msg_data[0];
//            *((uint8_t *)(&output) + 1) = msg_data[1];
//            *((uint8_t *)(&output) + 2) = msg_data[2];
//            *((uint8_t *)(&output) + 3) = msg_data[3];
//            return output;
//        }
//    }
}

uint32_t ODriveCAN::GetCurrentState() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

//    CAN_message_t return_msg;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

//    while (true) {
//        if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
//            memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
//            *((uint8_t *)(&output) + 0) = msg_data[4];
//            *((uint8_t *)(&output) + 1) = msg_data[5];
//            *((uint8_t *)(&output) + 2) = msg_data[6];
//            *((uint8_t *)(&output) + 3) = msg_data[7];
//            return output;
//        }
//    }
}

bool ODriveCAN::RunState(uint8_t requested_state) {
    sendMessage(CMD_ID_SET_AXIS_REQUESTED_STATE, false, 1, (byte*) &requested_state);
    return true;
}
