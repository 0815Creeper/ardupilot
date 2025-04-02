#pragma once

#include <stdint.h>
#include <stdio.h>
#include <mutex>

#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define SEND_SENSORDATA_BACK
#define IMU_BUFF_LEN 14*8
#define MAG_BUFF_LEN 12
#define GPS_BUFF_LEN 100
#define UDP_HIL_PORT 13017
#define UDP_HIL_RESPONSE_PORT (UDP_HIL_PORT + 1000)

struct DataStruct {
    float Baro_pressure;
    float Baro_temprature;
    uint8_t IMU_buff[IMU_BUFF_LEN];
    uint8_t MAG_buff[MAG_BUFF_LEN];
    uint8_t seq_num;
    uint8_t GPS_buff[GPS_BUFF_LEN];
};

class UDP_HIL {
public:
    static UDP_HIL& getInstance();

    void getInIMUbuff(uint8_t* buff);
    void getInMAGbuff(uint8_t* buff);
    void getInBaro(float* p, float* t);
    void setOutIMUbuff(uint8_t* buff);
    void setOutMAGbuff(uint8_t* buff);
    void setOutBaro(float p, float t);

    void _timer_tick();

private:
    UDP_HIL();
    ~UDP_HIL() = default;

    void init_socket();

    UDP_HIL(const UDP_HIL&) = delete;
    UDP_HIL& operator=(const UDP_HIL&) = delete;

    static uint8_t getSeq(const DataStruct& data);
    void getBaroFloatsFromDatastruct(float* p, float* t, uint8_t* newData);
    void setInData(uint8_t* newData);
    void setOutSeq(uint8_t s);
    void resetOutSeq();
    DataStruct getOutData();

    DataStruct in_data_{};
    DataStruct out_data_{};
    mutable std::mutex in_mutex_;
    mutable std::mutex out_mutex_;
    int udp_hil_socket = -1;
    bool socket_inited = false;
};
