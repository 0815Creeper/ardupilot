#pragma once

#include <stdint.h>
#include <stdio.h>
#include <mutex>
//#include "AP_Common/Location.h"
//#include "AP_GPS/AP_GPS.h"

#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "AP_HAL_Linux/PlantModel.h"

#define SEND_SENSORDATA_BACK
#define IMU_BUFF_LEN (3+1+3)*8
//#define UDP_HIL_GPS_BUFF_LEN 255
#define UDP_HIL_PORT 13017
#define UDP_HIL_RESPONSE_PORT (UDP_HIL_PORT + 1000)
//#define UDP_FAILSAVE_SWITCHBACK_MICROS 25000

struct GPSStruct {
    int32_t lat; // in 1E7 degrees
    int32_t lon; // in 1E7 degrees
    int32_t alt; // in cm
    float gspd; // m/s
    float gcourse; // Heading 2D deg * 100000
    Vector3f velocity;///< 3D velocity in m/s, in NED format
    float horizontal_accuracy;
    float vertical_accuracy;
    float speed_accuracy;
    uint16_t vDOP;
    uint16_t hDOP;
    uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
    uint16_t time_week;                 ///< GPS week number                      ///< horizontal dilution of precision in cm
    uint8_t status; //Ardupilot status, not UBX; see AP_GPS for definition
    uint8_t num_sats;
};

struct DataStruct {
    uint16_t motorPWM0;
    uint16_t motorPWM1;
    uint16_t motorPWM2;
    uint16_t motorPWM3;
    float Baro_pressure;
    float Baro_temprature;
    uint16_t IMU_buff[IMU_BUFF_LEN];
    Vector3f MAG_xyz;
    uint8_t seq_num;
    uint8_t gps_len;
    uint8_t gps_setup;
    GPSStruct GPSstate;
    //uint8_t GPS_buff[UDP_HIL_GPS_BUFF_LEN];
};

class UDP_HIL {
public:
    static UDP_HIL& getInstance();

    //void getPlantIMUbuff(uint16_t* buff, int16_t t2, uint8_t num_samples);
    Vector3f getPlantAccel();
    Vector3f getPlantGyro();

    bool getSwitchedOver() const { return switchedOver; }
    bool getUsePlantModel();

    uint8_t getInSeq();
    
    void setOutIMUbuff(uint8_t* buff, uint8_t n_samples);
    void setOutMAGbuff(Vector3f* buff);
    void setOutBaro(float p, float t);
    void setOutGPSstate(GPSStruct* buff);
    void setOutMotor(uint8_t ch, uint16_t pwm);
    
    void getInIMUbuff(uint8_t* buff);
    void getInMAGbuff(Vector3f* buff);
    GPSStruct getInGPSstate();
    void getInBaro(float* p, float* t);
    void getOutIMUbuff(uint8_t* buff);
    void getOutMAGbuff(Vector3f* buff);
    GPSStruct getOutGPSstate();
    void getOutBaro(float* p, float* t);

    //todo switchover func for all getIn Funcs

    // points to getOutGPSstate initally until udp stream is available
    void (UDP_HIL::*getValidIMUbuff)(uint8_t* buff);
    void (UDP_HIL::*getValidMAGbuff)(Vector3f* buff);
    GPSStruct (UDP_HIL::*getValidGPSstate)();
    void (UDP_HIL::*getValidBaro)(float* p, float* t);

    void _timer_tick();

private:
    UDP_HIL();
    ~UDP_HIL() = default;

    void init_socket();

    UDP_HIL(const UDP_HIL&) = delete;
    UDP_HIL& operator=(const UDP_HIL&) = delete;

    static uint8_t getSeq(const DataStruct& data);
    void setInData(struct DataStruct* newData);
    DataStruct getOutData();

    void inDataSwitchOver();
    //void inDataSwitchBack();

    DataStruct in_data_{};
    DataStruct out_data_{};
    mutable std::mutex in_mutex_;
    mutable std::mutex out_mutex_;

    bool switchedOver = false;
    
    int udp_hil_socket = -1;
    bool socket_inited = false;
    
    //uint32_t last_valid_udp_packet = 0;
    uint32_t last_debug_print = 0;

    PlantModel plant_model;
};
