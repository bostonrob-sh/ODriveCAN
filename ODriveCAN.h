#ifndef ODriveCAN_h
#define ODriveCAN_h

#include "Arduino.h"

typedef void (*send_callback)(uint32_t arbitration_id, uint8_t *data, uint8_t dlc, bool rtr);
typedef bool (*recv_callback)(uint32_t arbitration_id, uint8_t *data, uint8_t *dlc);

class ODriveCAN {
public:
    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };

    enum CommandId_t {
        CMD_ID_CANOPEN_NMT_MESSAGE = 0x000,
        CMD_ID_ODRIVE_HEARTBEAT_MESSAGE = 0x001,
        CMD_ID_ODRIVE_ESTOP_MESSAGE = 0x002,
        CMD_ID_GET_MOTOR_ERROR = 0x003,
        CMD_ID_GET_ENCODER_ERROR = 0x004,
        CMD_ID_GET_SENSORLESS_ERROR = 0x005,
        CMD_ID_SET_AXIS_NODE_ID = 0x006,
        CMD_ID_SET_AXIS_REQUESTED_STATE = 0x007,
        CMD_ID_SET_AXIS_STARTUP_CONFIG = 0x008,
        CMD_ID_GET_ENCODER_ESTIMATES = 0x009,
        CMD_ID_GET_ENCODER_COUNT = 0x00A,
        CMD_ID_SET_CONTROLLER_MODES = 0x00B,
        CMD_ID_SET_INPUT_POS = 0x00C,
        CMD_ID_SET_INPUT_VEL = 0x00D,
        CMD_ID_SET_INPUT_TORQUE = 0x00E,
        CMD_ID_SET_VELOCITY_LIMIT = 0x00F,
        CMD_ID_START_ANTICOGGING = 0x010,
        CMD_ID_SET_TRAJ_VEL_LIMIT = 0x011,
        CMD_ID_SET_TRAJ_ACCEL_LIMITS = 0x012,
        CMD_ID_SET_TRAJ_INERTIA = 0x013,
        CMD_ID_GET_IQ = 0x014,
        CMD_ID_GET_SENSORLESS_ESTIMATES = 0x015,
        CMD_ID_REBOOT_ODRIVE = 0x016,
        CMD_ID_GET_VBUS_VOLTAGE = 0x017,
        CMD_ID_CLEAR_ERRORS = 0x018,
        CMD_ID_CANOPEN_HEARTBEAT_MESSAGE = 0x700
    };

    ODriveCAN(uint8_t _axis_id, send_callback _send_cb, recv_callback _recv_cb):
      axis_id(_axis_id), send_cb(_send_cb), recv_cb(_recv_cb) {};

    void sendMessage(int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes);

    // Passive receiving from ODrive (non-blocking)
    void ReceiveHeartBeat();
    void ReceivePosVel();

    // Commands
    void SetPosition(float position);
    void SetPosition(float position, float velocity_feedforward);
    void SetPosition(float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(float velocity);
    void SetVelocity(float velocity, float current_feedforward);
    void SetVelocityLimit(float velocity_limit);
    void SetTorque(float torque);
    void ClearErrors();

    // Getters
    float GetPosition();
    float GetVelocity();
    uint32_t GetMotorError();
    uint32_t GetEncoderError();
    float GetIQMeasured();
    void GetIQ(float* iq_set, float* iq_measured);

    uint32_t GetAxisError();    // blocking ~100ms until the next heartbeat
    uint32_t GetCurrentState(); // --""--
    float GetVbusVoltage();

    // State helper
    bool RunState(uint8_t requested_state);

    uint32_t error, state;
    float pos, vel;

private:
    uint8_t axis_id;
    send_callback send_cb;
    recv_callback recv_cb;
    uint8_t _data_size;

};

#endif
