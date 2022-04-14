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
    int arbitration_id = (axis_id << CommandIDLength) + cmd_id;
    if (!remote_transmission_request) {
        send_cb(arbitration_id, signal_bytes, length, remote_transmission_request);
        return;
    }

    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    send_cb(arbitration_id, signal_bytes, length, remote_transmission_request);
    while (true) {
        if (recv_cb(arbitration_id, msg_data, &_data_size)) {
          memcpy(signal_bytes, msg_data, _data_size);
            return;
        }
    }
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

    sendMessage(CMD_ID_SET_INPUT_POS, false, 8, msg_data);
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
    
    sendMessage(CMD_ID_SET_INPUT_VEL, false, 8, msg_data);
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

    sendMessage(CMD_ID_GET_ENCODER_ESTIMATES, true, 0, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

float ODriveCAN::GetVelocity() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(CMD_ID_GET_ENCODER_ESTIMATES, true, 0, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[4];
    *((uint8_t *)(&output) + 1) = msg_data[5];
    *((uint8_t *)(&output) + 2) = msg_data[6];
    *((uint8_t *)(&output) + 3) = msg_data[7];
    return output;
}

void ODriveCAN::ReceivePosVel() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_GET_ENCODER_ESTIMATES;

    if (recv_cb(msg_id, msg_data, &_data_size)) {
        *((uint8_t *)&pos + 0) = msg_data[0];
        *((uint8_t *)&pos + 1) = msg_data[1];
        *((uint8_t *)&pos + 2) = msg_data[2];
        *((uint8_t *)&pos + 3) = msg_data[3];

        *((uint8_t *)&vel + 0) = msg_data[4];
        *((uint8_t *)&vel + 1) = msg_data[5];
        *((uint8_t *)&vel + 2) = msg_data[6];
        *((uint8_t *)&vel + 3) = msg_data[7];
    }
}

uint32_t ODriveCAN::GetMotorError() {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(CMD_ID_GET_MOTOR_ERROR, true, 0, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

uint32_t ODriveCAN::GetEncoderError() {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(CMD_ID_GET_ENCODER_ERROR, true, 0, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

void ODriveCAN::ReceiveHeartBeat() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

    if (recv_cb(msg_id, msg_data, &_data_size)) {
        *((uint8_t *)&error + 0) = msg_data[0];
        *((uint8_t *)&error + 1) = msg_data[1];
        *((uint8_t *)&error + 2) = msg_data[2];
        *((uint8_t *)&error + 3) = msg_data[3];

        *((uint8_t *)&state + 0) = msg_data[4];
        *((uint8_t *)&state + 1) = msg_data[5];
        *((uint8_t *)&state + 2) = msg_data[6];
        *((uint8_t *)&state + 3) = msg_data[7];
    }
}

float ODriveCAN::GetIQMeasured() {
    float iq_set, iq_measured;
    GetIQ(&iq_set, &iq_measured);
    return iq_measured;
}

void ODriveCAN::GetIQ(float* iq_set, float* iq_measured) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(CMD_ID_GET_IQ, true, 0, msg_data);

    *((uint8_t *)iq_set + 0) = msg_data[0];
    *((uint8_t *)iq_set + 1) = msg_data[1];
    *((uint8_t *)iq_set + 2) = msg_data[2];
    *((uint8_t *)iq_set + 3) = msg_data[3];

    *((uint8_t *)iq_measured + 0) = msg_data[4];
    *((uint8_t *)iq_measured + 1) = msg_data[5];
    *((uint8_t *)iq_measured + 2) = msg_data[6];
    *((uint8_t *)iq_measured + 3) = msg_data[7];
}

uint32_t ODriveCAN::GetAxisError() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

    while (true) {
        if (recv_cb(msg_id, msg_data, &_data_size)) {
            *((uint8_t *)(&output) + 0) = msg_data[0];
            *((uint8_t *)(&output) + 1) = msg_data[1];
            *((uint8_t *)(&output) + 2) = msg_data[2];
            *((uint8_t *)(&output) + 3) = msg_data[3];
            return output;
        }
    }
}

uint32_t ODriveCAN::GetCurrentState() {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

    while (true) {
        if (recv_cb(msg_id, msg_data, &_data_size)) {
            *((uint8_t *)(&output) + 0) = msg_data[4];
            *((uint8_t *)(&output) + 1) = msg_data[5];
            *((uint8_t *)(&output) + 2) = msg_data[6];
            *((uint8_t *)(&output) + 3) = msg_data[7];
            return output;
        }
    }
}

float ODriveCAN::GetVbusVoltage() {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(CMD_ID_GET_VBUS_VOLTAGE, true, 0, msg_data);

    float output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

bool ODriveCAN::RunState(uint8_t requested_state) {
    sendMessage(CMD_ID_SET_AXIS_REQUESTED_STATE, false, 1, (byte*) &requested_state);
    return true;
}
