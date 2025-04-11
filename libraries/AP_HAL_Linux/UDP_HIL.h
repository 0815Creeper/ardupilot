#pragma once

#include <stdint.h>
#include <stdio.h>
#include <mutex>

#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define SEND_SENSORDATA_BACK
#define IMU_BUFF_LEN 14*8
#define UDP_HIL_GPS_BUFF_LEN 255
#define UDP_HIL_PORT 13017
#define UDP_HIL_RESPONSE_PORT (UDP_HIL_PORT + 1000)

struct DataStruct {
    float Baro_pressure;
    float Baro_temprature;
    uint8_t IMU_buff[IMU_BUFF_LEN];
    Vector3f MAG_xyz;
    uint8_t seq_num;
    uint8_t gps_len;
    uint8_t gps_setup;
    uint8_t GPS_buff[UDP_HIL_GPS_BUFF_LEN];
};

class UDP_HIL {
public:
    static UDP_HIL& getInstance();
    uint8_t getInSeq();
    void getInIMUbuff(uint8_t* buff);
    void getInMAGbuff(Vector3f* buff);
    void getInGPSbuff(uint8_t* buff, uint8_t pos);
    void getLclGPSbuff(uint8_t* buff, uint8_t pos);
    void getInBaro(float* p, float* t);
    void setOutIMUbuff(uint8_t* buff, uint8_t n_samples);
    void setOutMAGbuff(Vector3f* buff);
    void setOutBaro(float p, float t);
    void setOutGPSbuff(uint8_t val, uint8_t pos);
    void setOutGPSsetup(uint8_t val);

    //todo switchover func for all getIn Funcs

    // points to getLclGPSbuff initally until gps is setup
    void (UDP_HIL::*getValidGPSbuff)(uint8_t* buff, uint8_t pos);

    void syncOutGPSbuff();
    
    void _timer_tick();

private:
    UDP_HIL();
    ~UDP_HIL() = default;

    void init_socket();

    UDP_HIL(const UDP_HIL&) = delete;
    UDP_HIL& operator=(const UDP_HIL&) = delete;

    static uint8_t getSeq(const DataStruct& data);
    void setInData(struct DataStruct* newData);
    void resetOutSeq();
    //void rotateFloats(struct DataStruct* d);
    DataStruct getOutData();

    void inDataSwitchOver();

    DataStruct in_data_{};
    DataStruct out_data_{};
    mutable std::mutex in_mutex_;
    mutable std::mutex out_mutex_;
    int udp_hil_socket = -1;
    bool socket_inited = false;
    uint8_t GPS_lcl_buff[UDP_HIL_GPS_BUFF_LEN];
    uint8_t GPS_lcl_buff_len;
    uint8_t GPS_lcl_buff_setup;
    bool switchedOver = false;
};
