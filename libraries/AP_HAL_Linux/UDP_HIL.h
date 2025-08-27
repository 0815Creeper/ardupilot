#pragma once

#include "AP_HAL_Linux/Experiments.h"
#include <stdint.h>
#include <stdio.h>
#include <mutex>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>

#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define UDP_HIL_IMU_BUFFER_LEN 32

#define SEND_SENSORDATA_BACK
#define UDP_HIL_PORT 13017
#define UDP_HIL_RESPONSE_PORT (UDP_HIL_PORT + 1000)

struct GPSStruct {
	int32_t  lat;                 // Latitude in 1E7 degrees
	int32_t  lon;                 // Longitude in 1E7 degrees
	int32_t  alt;                 // Altitude in cm
	float    gspd;                // Ground speed in m/s
	float    gcourse;             // Heading 2D in degrees*100000
	Vector3f velocity;            // 3D velocity in m/s, NED format
	float    horizontal_accuracy; // Horizontal position accuracy in meters
	float    vertical_accuracy;   // Vertical position accuracy in meters
	float    speed_accuracy;      // Speed accuracy in m/s
	uint16_t vDOP;                // Vertical dilution of precision in cm
	uint16_t hDOP;                // Horizontal dilution of precision in cm
	uint32_t time_week_ms;        // GPS time: ms since start of GPS week
	uint16_t time_week;           // GPS week number
	uint8_t  status;              // ArduPilot GPS status code (not UBX); \-see AP_GPS for definition
	uint8_t  num_sats;            // Number of satellites in view
};

struct DataStruct {
	uint16_t motorPWM0;           // PWM output for motor 0
	uint16_t motorPWM1;           // PWM output for motor 1
	uint16_t motorPWM2;           // PWM output for motor 2
	uint16_t motorPWM3;           // PWM output for motor 3
	float    Baro_pressure;       // Barometric pressure in Pascals
	float    Baro_temperature;    // Barometric temperature in degree Celsius
	uint16_t IMU_buff[MPU_SAMPLE_SIZE * UDP_HIL_IMU_BUFFER_LEN / sizeof(uint16_t)]; 
	Vector3f MAG_xyz;             // Magnetometer readings in uT (X, Y, Z)
	uint8_t  seq_num;             // Packet sequence number
	uint8_t  udp_hil_config;      // msb7...lsb0: bit1=START_EXPERIMENT (flag for UDP_HIL: set by simulation); bit0=UDP_HIL_RESPONSE_TYPE_ALWAYS (config_info for simulation: set by Ardupilot)
	uint8_t  unused_reserved;     // Reserved for alignment/future use
	GPSStruct GPSstate;           // GPS state data

    void reset() {
        // Reset individual members to zero 
        motorPWM0 = 0;
        motorPWM1 = 0;
        motorPWM2 = 0;
        motorPWM3 = 0;
        Baro_pressure = 0.0f;
        Baro_temperature = 0.0f;
        memset(IMU_buff, 0, sizeof(IMU_buff));
        MAG_xyz.x = 0.0f;
        MAG_xyz.y = 0.0f;
        MAG_xyz.z = 0.0f;
        seq_num = 0;
        udp_hil_config = 0;
        unused_reserved = 0;
        GPSstate.lat = 0;
        GPSstate.lon = 0;
        GPSstate.alt = 0;
        GPSstate.gspd = 0.0f;
        GPSstate.gcourse = 0.0f;
        GPSstate.velocity.x = 0.0f;
        GPSstate.velocity.y = 0.0f;
        GPSstate.velocity.z = 0.0f;
        GPSstate.horizontal_accuracy = 0.0f;
        GPSstate.vertical_accuracy = 0.0f;
        GPSstate.speed_accuracy = 0.0f;
        GPSstate.vDOP = 0;
        GPSstate.hDOP = 0;
        GPSstate.time_week_ms = 0;
        GPSstate.time_week = 0;
        GPSstate.status = 0;
        GPSstate.num_sats = 0;
    }
};

class UDP_HIL {
public:
    static UDP_HIL& getInstance();

    bool getSwitchedOver() const { return switchedOver; }

    uint8_t getInSeq();

    void setOutIMUbuff(uint8_t* buff, uint8_t offset8, uint8_t n_samples);
    void setOutMAGbuff(Vector3f* buff);
    void setOutBaro(float p, float t);
    void setOutGPSstate(GPSStruct* buff);
    void setOutMotor(uint8_t ch, uint16_t pwm);
    
    void getInIMUbuff(uint8_t* buff, uint8_t offset8);
    void getInMAGbuff(Vector3f* buff);
    GPSStruct getInGPSstate();
    void getInBaro(float* p, float* t);
    void getOutIMUbuff(uint8_t* buff, uint8_t offset8);
    void getOutMAGbuff(Vector3f* buff);
    GPSStruct getOutGPSstate();
    void getOutBaro(float* p, float* t);

    int16_t getInIMUgx();
    bool getInStartExperiment();

    // points to getOutGPSstate initally until udp stream is available
    void (UDP_HIL::*getValidIMUbuff)(uint8_t* buff, uint8_t offset8);
    void (UDP_HIL::*getValidMAGbuff)(Vector3f* buff);
    GPSStruct (UDP_HIL::*getValidGPSstate)();
    void (UDP_HIL::*getValidBaro)(float* p, float* t);

    void _timer_tick();

private:
    UDP_HIL();
    ~UDP_HIL() = default;

    void init_socket();
    struct sockaddr_in response_addr;

    UDP_HIL(const UDP_HIL&) = delete;
    UDP_HIL& operator=(const UDP_HIL&) = delete;

    static uint8_t getSeq(const DataStruct& data);
    void setInData(struct DataStruct* newData);
    void addInSeq();
    DataStruct getOutData();

    void inDataSwitchOver();

    DataStruct in_data_{};
    DataStruct out_data_{};
    mutable std::mutex in_mutex_;
    mutable std::mutex out_mutex_;

    bool switchedOver = false;
    
    int udp_hil_socket = -1;
    bool socket_inited = false;

    uint32_t last_debug_print = 0;
    uint32_t packets_send = 0;
    uint32_t packets_recv = 0;
    uint32_t packets_recv_B = 0;
    uint32_t zero_packets_in_os_queue_cnt_B = 0;
    uint32_t one_packets_in_os_queue_cnt_B = 0;
    uint32_t more_packets_in_os_queue_cnt_B = 0;
    uint32_t tick_calls_total = 0;
    uint32_t tick_calls_since_init = 0;


    void setOutGPSTOW(uint32_t t);

    #ifdef UDP_HIL_RESET_AFTER_MILLIS
    uint32_t last_valid_packet = 0;
    #endif

    #ifdef TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
    volatile uint32_t* gpio_map = nullptr;
    #endif
};
