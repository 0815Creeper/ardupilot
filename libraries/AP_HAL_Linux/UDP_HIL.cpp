#include <AP_HAL_Linux/Experiments.h>

#ifdef UDP_HIL_ENABLED

#include <unistd.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "UDP_HIL.h"


#if defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_THREAD) 
#include <time.h>
struct timespec UDP_HIL_TimeOperation_ts;
struct timespec UDP_HIL_TimeOperation_tsp;
#endif
#ifdef TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION
#include <time.h>
struct timespec UDP_HIL_tick_nanos;
struct timespec UDP_HIL_prev_tick_nanos;
#endif

UDP_HIL& UDP_HIL::getInstance() {
    static UDP_HIL instance;
    return instance;
}

UDP_HIL::UDP_HIL() {
    getValidIMUbuff =&UDP_HIL::getOutIMUbuff;
    getValidMAGbuff =&UDP_HIL::getOutMAGbuff;
    getValidGPSstate =&UDP_HIL::getOutGPSstate;
    getValidBaro =&UDP_HIL::getOutBaro;
    init_socket();
}

void UDP_HIL::init_socket() {
    printf("UDP_HIL_creation, PID:%i\n", getpid());
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

void UDP_HIL::setInData(struct DataStruct* newData) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy((&in_data_), newData, sizeof(struct DataStruct));
}

DataStruct UDP_HIL::getOutData() {
    std::lock_guard<std::mutex> lock(out_mutex_);
    return out_data_;
}

uint8_t UDP_HIL::getSeq(const DataStruct& data) {
    return data.seq_num;
}

uint8_t UDP_HIL::getInSeq(){
    std::lock_guard<std::mutex> lock(in_mutex_);
    return in_data_.seq_num;
}

void UDP_HIL::getInBaro(float* p, float* t) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    *p = in_data_.Baro_pressure;
    *t = in_data_.Baro_temprature;
}
void UDP_HIL::getOutBaro(float* p, float* t) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    *p = out_data_.Baro_pressure;
    *t = out_data_.Baro_temprature;
}

void UDP_HIL::getInIMUbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy(buff, in_data_.IMU_buff, MPU_SAMPLE_SIZE*MPU_FIFO_BUFFER_LEN);
}
void UDP_HIL::getOutIMUbuff(uint8_t* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(buff, out_data_.IMU_buff, MPU_SAMPLE_SIZE*MPU_FIFO_BUFFER_LEN);
}

void UDP_HIL::getInMAGbuff(Vector3f* buff) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy(buff, &in_data_.MAG_xyz, sizeof(Vector3f));
}
void UDP_HIL::getOutMAGbuff(Vector3f* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(buff, &out_data_.MAG_xyz, sizeof(Vector3f));
}

GPSStruct UDP_HIL::getOutGPSstate() {
    std::lock_guard<std::mutex> lock(out_mutex_);
    return out_data_.GPSstate;
}

GPSStruct UDP_HIL::getInGPSstate() {
    std::lock_guard<std::mutex> lock(in_mutex_);
    return in_data_.GPSstate;
}

void UDP_HIL::setOutBaro(float p, float t) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    out_data_.Baro_pressure = p;
    out_data_.Baro_temprature = t;
}

void UDP_HIL::setOutIMUbuff(uint8_t* buff, uint8_t n_samples) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    uint16_t n = n_samples*MPU_SAMPLE_SIZE;
    memcpy(out_data_.IMU_buff, buff, n);
    memset(((uint8_t*)out_data_.IMU_buff) + n, 0, MPU_SAMPLE_SIZE*MPU_FIFO_BUFFER_LEN-n);
}

void UDP_HIL::setOutMAGbuff(Vector3f* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(&out_data_.MAG_xyz, buff, sizeof(out_data_.MAG_xyz));
}

void UDP_HIL::setOutGPSstate(GPSStruct* buff) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(&out_data_.GPSstate, buff, sizeof(struct GPSStruct));
}

void UDP_HIL::setOutMotor(uint8_t ch, uint16_t pwm) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    switch(ch) {
        case 0:
            out_data_.motorPWM0 = pwm;
            break;
        case 1:
            out_data_.motorPWM1 = pwm;
            break;
        case 2:
            out_data_.motorPWM2 = pwm;
            break;
        case 3:
            out_data_.motorPWM3 = pwm;
            break;
    }
}

void UDP_HIL::inDataSwitchOver() {
    switchedOver = true;
    printf("UDP_HIL_SWITCH_OVER!!!\n");
    getValidGPSstate = &UDP_HIL::getInGPSstate;
    getValidBaro = &UDP_HIL::getInBaro;
    getValidIMUbuff = &UDP_HIL::getInIMUbuff;
    getValidMAGbuff = &UDP_HIL::getInMAGbuff;
}

//#include <unistd.h>     //getpid()
//#include <pthread.h>    //pthread_self()

void UDP_HIL::_timer_tick() {
    #ifdef TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION
    clock_gettime(CLOCK_MONOTONIC, &UDP_HIL_tick_nanos);
    uint64_t dt = (uint64_t)(UDP_HIL_tick_nanos.tv_sec - UDP_HIL_prev_tick_nanos.tv_sec) * (uint64_t)1000000000UL + (uint64_t)(UDP_HIL_tick_nanos.tv_nsec - UDP_HIL_prev_tick_nanos.tv_nsec);
    UDP_HIL_prev_tick_nanos = UDP_HIL_tick_nanos;
    TIMING_EXPERIMENT_UDP_HIL_OUTPUT(dt);
    #endif
    
    if(!socket_inited){
        printf("udp_hil_tick_but_socket_not_rdy\n");
        init_socket();
    }
    
    //printf("PID: %d, Thread-ID: %lu, udp_hil_socket: %d\n", getpid(), pthread_self(), udp_hil_socket);
        
    struct sockaddr_in client_addr, response_addr;
    socklen_t addr_len = sizeof(client_addr);
    struct DataStruct buffer;
    if (udp_hil_socket < 0) {
        printf("Socket ist ungültig, kann nicht verwenden.\n");
        while(1);
    }

    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM
    clock_gettime(CLOCK_MONOTONIC, &UDP_HIL_TimeOperation_ts);
    #endif
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &UDP_HIL_TimeOperation_ts);
    #endif
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &UDP_HIL_TimeOperation_ts);
    #endif
    ssize_t recv_len = recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&client_addr, &addr_len);
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM
    clock_gettime(CLOCK_MONOTONIC, &UDP_HIL_TimeOperation_tsp);
    #endif
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &UDP_HIL_TimeOperation_tsp);
    #endif
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &UDP_HIL_TimeOperation_tsp);
    #endif
    if (recv_len == -1){
        if (switchedOver&&AP_HAL::micros()-last_debug_print>1000) {
            last_debug_print=AP_HAL::micros();
            printf("ONE UDP_HIL timer_tick_skipped at %i\n", AP_HAL::micros());
        }
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
        if(!((getSeq(in_data_) == 255 && getSeq(buffer) == 1) || getSeq(buffer) == getSeq(in_data_) + 1 || getSeq(buffer) == 0)){
            printf("FEHLER SEQ NUM FOLGE: lseq:%i seq:%i \n", getSeq(in_data_), getSeq(buffer));
            exit(1);
        }
        setInData(&buffer);
        if (!switchedOver && getSeq(in_data_) > 0)
            inDataSwitchOver();

        response_addr.sin_family = AF_INET;
        response_addr.sin_addr = client_addr.sin_addr;
        response_addr.sin_port = htons(UDP_HIL_RESPONSE_PORT);
        buffer = getOutData();
        buffer.seq_num = getInSeq() + 1;
        
        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM
        clock_gettime(CLOCK_MONOTONIC, &UDP_HIL_TimeOperation_ts);
        #endif
        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_PROCESS
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &UDP_HIL_TimeOperation_ts);
        #endif
        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_THREAD
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &UDP_HIL_TimeOperation_ts);
        #endif

        ssize_t sent_len = sendto(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&response_addr, addr_len);

        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM
        clock_gettime(CLOCK_MONOTONIC, &UDP_HIL_TimeOperation_tsp);
        #endif
        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_PROCESS
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &UDP_HIL_TimeOperation_tsp);
        #endif
        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_THREAD
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &UDP_HIL_TimeOperation_tsp);
        #endif

        if (sent_len < 0) {
            perror("Fehler beim Senden der Antwort");
        }

        #if defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM) || defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_PROCESS) || defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_THREAD) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_THREAD)
        int64_t UDP_HIL_TimeOperation_difference = (int64_t)(UDP_HIL_TimeOperation_tsp.tv_sec - UDP_HIL_TimeOperation_ts.tv_sec) * (int64_t)1000000000UL + (int64_t)(UDP_HIL_TimeOperation_tsp.tv_nsec - UDP_HIL_TimeOperation_ts.tv_nsec);
        TIMING_EXPERIMENT_UDP_HIL_OUTPUT(UDP_HIL_TimeOperation_difference);
        #endif    
    } else {
        printf("FEHLER RCV_LEN: sizeof(buffer):%i recv_len:%i \n", sizeof(buffer), recv_len);
        exit(1);
    }
    while(recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len) != -1)
        printf("UDP_HIL skipping packet\n");
}

#endif // UDP_HIL_ENABLED