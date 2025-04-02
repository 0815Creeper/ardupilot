#include "UDP_HIL.h"
#include <AP_HAL/AP_HAL.h>

UDP_HIL& UDP_HIL::getInstance() {
    static UDP_HIL instance;
    return instance;
}

UDP_HIL::UDP_HIL() {
    init_socket();
}

void UDP_HIL::init_socket() {
    printf("UDP_HIL_creation\n");
    struct sockaddr_in server_addr;
    udp_hil_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_hil_socket < 0) {
        perror("Fehler beim Erstellen des Sockets");
        printf("Fehler beim Erstellen des Sockets\n");
        while(1);
    }
    // Nicht-blockierenden Modus aktivieren
    int flags = fcntl(udp_hil_socket, F_GETFL, 0);
    if (flags == -1) {
        perror("Fehler beim Abrufen der Socket-Flags");
        printf("Fehler beim Abrufen der Socket-Flags\n");
        while(1);
    }
    if (fcntl(udp_hil_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("Fehler beim Setzen des non-blocking Modus");
        printf("Fehler beim Setzen des non-blocking Modus\n");
        while(1);
    }
    // Serveradresse konfigurieren
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_HIL_PORT);
    // Socket an Port binden
    if (bind(udp_hil_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(udp_hil_socket);
        udp_hil_socket = -1;
        AP_HAL::panic("Socket Bind fehlgeschlagen");
        while(1);
    } else {
        printf("UDP_HIL: UDP-Socket initialisiert (Port %d, non-blocking), micros: %u\n", UDP_HIL_PORT, AP_HAL::micros());
        socket_inited = true;
    }
}

void UDP_HIL::setInData(uint8_t* newData) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy((&in_data_), newData, sizeof(struct DataStruct));
    getBaroFloatsFromDatastruct(&in_data_.Baro_pressure, &in_data_.Baro_temprature, newData);
    //printf("cpy size: %i\n",sizeof(struct DataStruct));
    //printf("seq_num_copied: %i\n", newData[132]);
    //printf("seq_num_indata: %i\n", in_data_.seq_num);
    //printf("in_data:\n");
    //for(int i=0;i<236;i++){    printf("%i ", in_data_.IMU_buff[i-8]);    }
    //printf("\nnewdata:\n");
    //for(int i=0;i<236;i++){    printf("%i ", newData[i]);    }
    //printf("end:\n");
}

void UDP_HIL::getBaroFloatsFromDatastruct(float* p, float* t, uint8_t* newData){
    uint32_t tmp;
    tmp = (uint32_t)(newData[0]<<24)|(newData[1]<<16)|(newData[2]<<8)|newData[3];
    memcpy(p, &tmp, sizeof(float));
    tmp = (uint32_t)(newData[4]<<24)|(newData[5]<<16)|(newData[6]<<8)|newData[7];
    memcpy(t, &tmp, sizeof(float));
}

DataStruct UDP_HIL::getOutData() {
    std::lock_guard<std::mutex> lock(out_mutex_);
    return out_data_;
}

uint8_t UDP_HIL::getSeq(const DataStruct& data) {
    return data.seq_num;
}

void UDP_HIL::getInBaro(float* p, float* t) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    *p = in_data_.Baro_pressure;
    *t = in_data_.Baro_temprature;
}

void UDP_HIL::getInIMUbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy(buff, in_data_.IMU_buff, IMU_BUFF_LEN);
}

void UDP_HIL::getInMAGbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy(buff, in_data_.MAG_buff, MAG_BUFF_LEN);
}

void UDP_HIL::setOutSeq(uint8_t s) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    out_data_.seq_num |= s;
}

void UDP_HIL::resetOutSeq() {
    std::lock_guard<std::mutex> lock(out_mutex_);
    out_data_.seq_num = 0;
}

void UDP_HIL::setOutBaro(float p, float t) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    out_data_.Baro_pressure = p;
    out_data_.Baro_temprature = t;
}

void UDP_HIL::setOutIMUbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(out_data_.IMU_buff, buff, IMU_BUFF_LEN);
}

void UDP_HIL::setOutMAGbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(out_data_.MAG_buff, buff, MAG_BUFF_LEN);
}

#include <unistd.h>     //getpid()
#include <pthread.h>    //pthread_self()

void UDP_HIL::_timer_tick() {
    if(!socket_inited){
        printf("udp_hil_tick_but_socket_not_rdy\n");
        init_socket();
    }
    //printf("PID: %d, Thread-ID: %lu, udp_hil_socket: %d\n", getpid(), pthread_self(), udp_hil_socket);
    struct sockaddr_in client_addr, response_addr;
    socklen_t addr_len = sizeof(client_addr);
    struct DataStruct buffer;
    //printf("trying-recv: size %i bytes: %i\n", sizeof(buffer), sizeof(in_data_.Baro_pressure));
    if (udp_hil_socket < 0) {
        printf("Socket ist ungültig, kann nicht verwenden.\n");
        while(1);
    }
    ssize_t recv_len = recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&client_addr, &addr_len);
    if (recv_len == -1){
        //TODO not good but for testing
        if (errno == EAGAIN || errno == EWOULDBLOCK) { //|| AP_HAL::micros()<10000000UL
            // Keine Daten empfangen, einfach weiter machen
            return;
        } else {
            perror("Fehler beim Empfangen von Daten");
            socket_inited = false;
            return;
        }
    } else if (recv_len == sizeof(buffer)) {
        if(!((getSeq(in_data_) == 255 && getSeq(buffer) == 1) || getSeq(buffer) == getSeq(in_data_) + 1)){
            printf("FEHLER SEQ NUM FOLGE\n");
            printf("FEHLER SEQ NUM FOLGE: lseq:%i seq:%i \n", getSeq(in_data_), getSeq(buffer));
            printf("FEHLER SEQ NUM FOLGE\n");
            while(1);
        }
        setInData((uint8_t*)&buffer);
        response_addr.sin_family = AF_INET;
        response_addr.sin_addr = client_addr.sin_addr;
        response_addr.sin_port = htons(UDP_HIL_RESPONSE_PORT);
        buffer = getOutData();
        ssize_t sent_len = sendto(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&response_addr, addr_len);
        if (sent_len < 0) {
            perror("Fehler beim Senden der Antwort");
        } 
            
        resetOutSeq();
    } else {
        printf("FEHLER RCV_LEN\n");
        printf("FEHLER RCV_LEN: sizeof(buffer):%i recv_len:%i \n", sizeof(buffer), recv_len);
        printf("FEHLER RCV_LEN\n");
        while(1);
    }

    //if (AP_HAL::micros()<8000000)
    //    printf("UDP_HIL_tick: %u\n", AP_HAL::micros());
}