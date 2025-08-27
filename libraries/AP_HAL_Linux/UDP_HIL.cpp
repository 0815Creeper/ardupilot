#include "AP_HAL_Linux/Experiments.h"

#ifdef UDP_HIL_ENABLED

#include <unistd.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "UDP_HIL.h"


#if defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM) 
    #include <time.h>
    struct timespec UDP_HIL_TimeOperation_ts;
    struct timespec UDP_HIL_TimeOperation_tsp;
#endif
#if defined(TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION) || defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DONE_PRECISION)
    #include <time.h>
    struct timespec UDP_HIL_tick_nanos;
    struct timespec UDP_HIL_prev_tick_nanos;
#endif
#ifdef TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
    #include "Experiments_output_GPIO.h"
#endif

UDP_HIL& UDP_HIL::getInstance() {
    static UDP_HIL instance;
    return instance;
}

UDP_HIL::UDP_HIL() {
    printf("UDP_HIL_constructor, RATE: %i\n", UDP_HIL_FREQ);
    printf("UDP_HIL_Size_datastruct: %i\n", sizeof(DataStruct));
    #ifdef UDP_HIL_RESPONSE_TYPE_ALWAYS
        printf("UDP_HIL_response_type: ALWAYS, start after: %dticks after switchover\n", UDP_HIL_RESPONSE_TYPE_ALWAYS_START_AFTER);
    #else
        printf("UDP_HIL_response_type: ON_RECIVE_ONLY\n");
    #endif
    #ifdef UDP_HIL_DISABLE_SEQ_CHECK
        printf("UDP_HIL: SEQ_CHECK DISABLED!!!\n");
    #endif
    #ifdef TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
        INIT_GPIO();
        GPIO_SET_OUTPUT_26();
    #endif    
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

void UDP_HIL::addInSeq() {
    std::lock_guard<std::mutex> lock(in_mutex_);
    if (in_data_.seq_num == 255) 
        in_data_.seq_num = 0;
    in_data_.seq_num++;
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
    *t = in_data_.Baro_temperature;
}
void UDP_HIL::getOutBaro(float* p, float* t) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    *p = out_data_.Baro_pressure;
    *t = out_data_.Baro_temperature;
}

void UDP_HIL::getInIMUbuff(uint8_t* buff, uint8_t offset8) {
    std::lock_guard<std::mutex> lock(in_mutex_);
    memcpy(buff, in_data_.IMU_buff + offset8 * MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN, MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN);
}
void UDP_HIL::getOutIMUbuff(uint8_t* buff, uint8_t offset8) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    memcpy(buff, out_data_.IMU_buff + offset8 * MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN, MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN);
}

int16_t UDP_HIL::getInIMUgx() {
    std::lock_guard<std::mutex> lock(in_mutex_);
    return (in_data_.IMU_buff[5] << 8) | (in_data_.IMU_buff[5] >> 8);
}
bool UDP_HIL::getInStartExperiment() {
    std::lock_guard<std::mutex> lock(in_mutex_);
    return (in_data_.udp_hil_config & 0x02) != 0;
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
    out_data_.Baro_temperature = t;
}

void UDP_HIL::setOutIMUbuff(uint8_t* buff, uint8_t offset8, uint8_t n_samples) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    uint16_t n = n_samples*MPU_SAMPLE_SIZE;
    if (offset8 == 0) {
        memset(out_data_.IMU_buff, 0, MPU_SAMPLE_SIZE * UDP_HIL_IMU_BUFFER_LEN);
    }
    memcpy(((uint8_t*)out_data_.IMU_buff) + offset8 * MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN, buff, n);
    memset(((uint8_t*)out_data_.IMU_buff) + offset8 * MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN + n, 0, MPU_SAMPLE_SIZE*MPU_FIFO_BUFFER_LEN*((UDP_HIL_IMU_BUFFER_LEN/MPU_FIFO_BUFFER_LEN)-offset8) - n);
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
    uint8_t buff_or = 0;
    uint8_t buffer[MPU_SAMPLE_SIZE * MPU_FIFO_BUFFER_LEN];
    getInIMUbuff(buffer, 0);
    for(int j=0; j<MPU_SAMPLE_SIZE; j++) {
        buff_or |= buffer[MPU_SAMPLE_SIZE+j];
    }
    if (!buff_or) {
        printf("UDP_HIL_switchover prevented by invalid IMU data\n");
        return;
    }
    switchedOver = true;
    printf("UDP_HIL_SWITCH_OVER!!!\n");
    getValidGPSstate = &UDP_HIL::getInGPSstate;
    getValidBaro = &UDP_HIL::getInBaro;
    getValidIMUbuff = &UDP_HIL::getInIMUbuff;
    getValidMAGbuff = &UDP_HIL::getInMAGbuff;
}

void UDP_HIL::setOutGPSTOW(uint32_t t) {
    std::lock_guard<std::mutex> lock(out_mutex_);
    out_data_.GPSstate.time_week_ms = t;
}

void UDP_HIL::_timer_tick() {
    #if defined(TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION) || defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DONE_PRECISION)
        uint64_t dt;
    #endif
    #ifdef TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION
        clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_tick_nanos);
        #ifdef TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
            GPIO_WRITE_26_0(); 
        #endif
        dt = (uint64_t)(UDP_HIL_tick_nanos.tv_sec - UDP_HIL_prev_tick_nanos.tv_sec) * (uint64_t)1000000000UL + (uint64_t)(UDP_HIL_tick_nanos.tv_nsec - UDP_HIL_prev_tick_nanos.tv_nsec);
        UDP_HIL_prev_tick_nanos = UDP_HIL_tick_nanos;
        TIMING_EXPERIMENT_UDP_HIL_OUTPUT(dt);
        #ifdef TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
            GPIO_WRITE_26_1();
        #endif
    #endif
    
    tick_calls_since_init++;
    tick_calls_total++;

    #ifdef UDP_HIL_RESET_AFTER_MILLIS
        if(last_valid_packet !=0 && AP_HAL::millis()-last_valid_packet > UDP_HIL_RESET_AFTER_MILLIS){
            printf("UDP_HIL: reset after %i ms\n", UDP_HIL_RESET_AFTER_MILLIS);
            last_valid_packet = 0;
            tick_calls_since_init = 0;
            close(udp_hil_socket);
            udp_hil_socket = -1;
            socket_inited = false;
            getValidIMUbuff =&UDP_HIL::getOutIMUbuff;
            getValidMAGbuff =&UDP_HIL::getOutMAGbuff;
            getValidGPSstate =&UDP_HIL::getOutGPSstate;
            getValidBaro =&UDP_HIL::getOutBaro;
            switchedOver = false;
            std::lock_guard<std::mutex> lock(in_mutex_);
            in_data_.reset();
            init_socket();
        }
    #endif

    if(!socket_inited){
        printf("udp_hil_tick_but_socket_not_rdy\n");
        init_socket();
    }
    
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    struct DataStruct buffer;
    if (udp_hil_socket < 0) {
        printf("Socket ist ungültig, kann nicht verwenden.\n");
        while(1);
    }

    #if defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM)
        clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_TimeOperation_ts);
    #endif
    #ifdef TIMING_EXPERIMENT_UDP_HIL_PINGPONG_DURATION_SYSTEM
        struct timespec startWaitTS;
        uint64_t apt=AP_HAL::micros();
        clock_gettime(CLOCK_MONOTONIC_RAW, &startWaitTS);
        uint64_t loops=0;
        ssize_t recv_len;
        while((recv_len=recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0,(struct sockaddr *)&client_addr, &addr_len))<=0&&switchedOver&&(AP_HAL::micros()-apt)<10000){
            loops++;
        }
        uint64_t dt = GET_TIME_NS(startWaitTS);
        clock_gettime(CLOCK_MONOTONIC_RAW, &startWaitTS);
        dt = GET_TIME_NS(startWaitTS) - dt;
        setOutGPSTOW(dt);
    #else

// here the reciving happens during usual operation
        ssize_t recv_len = recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&client_addr, &addr_len);
// 

    #endif
    #ifdef  TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM
        clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_TimeOperation_tsp);
    #endif
    #ifdef TIMING_EXPERIMENT_UDP_HIL_PINGPONG_DURATION_SYSTEM
        if(dt>1000000){
            printf("UDP_HIL: PINGPONG ERROR: dt: %lli, ap_micro_diff: %lli, prev seq: %i\n", dt, AP_HAL::micros()-apt, getSeq(in_data_));
            //exit(1);
        }
    #endif

    uint32_t total_tmp = tick_calls_since_init % UDP_HIL_DEBUG_PACKET_STATS_INTERVAL;
    #ifdef UDP_HIL_DEBUG_PACKET_STATS_INTERVAL_PRINT
        if (total_tmp == (UDP_HIL_DEBUG_PACKET_STATS_INTERVAL - 1)){
            printf("UDP_HIL: rcv_timeouts: %.2f%%, rcv_timeouts_%.1fs: %.2f%%, packets_recv: %u, packets_send: %u, tick_calls_total: %u, tick_calls_since_init: %u, num_packets_in_os_queue: %u, %u, %u\n", 100*(1-(float)packets_recv/(float)tick_calls_since_init), (float)UDP_HIL_DEBUG_PACKET_STATS_INTERVAL/(float)UDP_HIL_FREQ,100*(1-(float)packets_recv_B/(float)(total_tmp?total_tmp:1)), packets_recv, packets_send, tick_calls_total, tick_calls_since_init, zero_packets_in_os_queue_cnt_B, one_packets_in_os_queue_cnt_B, more_packets_in_os_queue_cnt_B   );
        }
    #endif 
    if (total_tmp == 0){
        packets_recv_B = 0;
        zero_packets_in_os_queue_cnt_B = 0;
        one_packets_in_os_queue_cnt_B = 0;
        more_packets_in_os_queue_cnt_B = 0;
    }
// checking for recive status (no packet, error or success)
    if (recv_len == -1){ // no packet recived
        #if defined(TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION) && defined(TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED)
            GPIO_WRITE_26_1();
        #endif
        // print timout status if no new packet was recived, APs calculations continue on old data still in Datastruct
        if (switchedOver) {
            #ifdef UDP_HIL_DEBUG_PACKET_TIMOUTS_AND_SKIPS
                printf("UDP_HIL: TIMEOUT: rcv_timeouts: %.2f%%, rcv_timeouts_10s: %.2f%%, packets_recv: %u, packets_send: %u, tick_calls_total: %u, tick_calls_since_init: %u\n", 100*(1-(float)packets_recv/(float)tick_calls_since_init), 100*(1-(float)packets_recv_B/(float)(total_tmp?total_tmp:1)), packets_recv, packets_send, tick_calls_total, tick_calls_since_init);
            #endif
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Keine Daten empfangen, einfach weiter machen
            #ifdef UDP_HIL_RESPONSE_TYPE_ALWAYS
                if(tick_calls_since_init < UDP_HIL_RESPONSE_TYPE_ALWAYS_START_AFTER)
                    return;
            #else
                return;
            #endif
        } else {
            perror("Fehler beim Empfangen von Daten");
            socket_inited = false;
            return;
        }
    } else if (recv_len == sizeof(buffer)) { // successful recive
        packets_recv++;
        packets_recv_B++;
        if(!(getSeq(buffer) == getSeq(in_data_) + 1 || (getSeq(in_data_) == 255 && getSeq(buffer) == 1) || getSeq(buffer) == 0)){
            #ifdef UDP_HIL_DISABLE_SEQ_CHECK
                printf("UDP_HIL: SEQ_CHECK DISABLED, but got wrong seq num: %i, expected: %i\n", getSeq(buffer), getSeq(in_data_) + 1);
            #else
                printf("FEHLER SEQ NUM FOLGE: lseq:%i seq:%i \n", getSeq(in_data_), getSeq(buffer));
                exit(1);
                return;
            #endif
        }
        last_valid_packet = AP_HAL::millis();
        setInData(&buffer);
        if (!switchedOver && getSeq(in_data_) > 0) {
            inDataSwitchOver();
        } else if (!switchedOver) {
            printf("UDP_HIL: INITAL_PACKET SEQ 0\n");
            packets_send = 0;
            packets_recv = 1;
            packets_recv_B = 1;
            tick_calls_since_init = 1;
            zero_packets_in_os_queue_cnt_B = 0;
            one_packets_in_os_queue_cnt_B = 0;
            more_packets_in_os_queue_cnt_B = 0;
            response_addr.sin_family = AF_INET;
            response_addr.sin_addr = client_addr.sin_addr;
            response_addr.sin_port = htons(UDP_HIL_RESPONSE_PORT);
        }
    } else { // error, got wrong package len
        printf("FEHLER RCV_LEN: sizeof(buffer):%i recv_len:%i \n", sizeof(buffer), recv_len);
        exit(1);
        return;
    }

    if(last_valid_packet != 0) {
        buffer = getOutData();
        buffer.seq_num = getInSeq() + 1;
        #ifndef UDP_HIL_RESPONSE_TYPE_ALWAYS
            buffer.udp_hil_config &= 0b11111110;
        #else         
            buffer.udp_hil_config |= 0b00000001;
        #endif

        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM
            clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_TimeOperation_ts);
        #endif

        ssize_t sent_len = sendto(udp_hil_socket, &buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&response_addr, addr_len);

        #ifdef TIMING_EXPERIMENT_UDP_HIL_SENDTO_DONE_PRECISION
            clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_tick_nanos);
            dt = (uint64_t)(UDP_HIL_tick_nanos.tv_sec - UDP_HIL_prev_tick_nanos.tv_sec) * (uint64_t)1000000000UL + (uint64_t)(UDP_HIL_tick_nanos.tv_nsec - UDP_HIL_prev_tick_nanos.tv_nsec);
            UDP_HIL_prev_tick_nanos = UDP_HIL_tick_nanos;
            TIMING_SENDTO_DONE_PRECISION_UDP_HIL_OUTPUT(dt);
        #endif

        #if defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM)
            clock_gettime(CLOCK_MONOTONIC_RAW, &UDP_HIL_TimeOperation_tsp);
        #endif

        if (sent_len < 0) {
            perror("Fehler beim Senden der Antwort");
        } else {
            packets_send++;
        }

        #if defined(TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM) || defined(TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM)
            int64_t UDP_HIL_TimeOperation_difference = (int64_t)(UDP_HIL_TimeOperation_tsp.tv_sec - UDP_HIL_TimeOperation_ts.tv_sec) * (int64_t)1000000000UL + (int64_t)(UDP_HIL_TimeOperation_tsp.tv_nsec - UDP_HIL_TimeOperation_ts.tv_nsec);
            TIMING_EXPERIMENT_UDP_HIL_OUTPUT(UDP_HIL_TimeOperation_difference);
        #endif 
    }

    /* skip packets if too many avail (only in mode where we try to force working syncronously) 
    while(recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len) != -1){
        #ifdef UDP_HIL_DEBUG_PACKET_TIMOUTS_AND_SKIPS
            printf("UDP_HIL skipping packet\n");
        #endif
        addInSeq();
    }*/
        
    unsigned int pending_bytes;
    ioctl(udp_hil_socket, FIONREAD, &pending_bytes);
    if(switchedOver){
        if (pending_bytes == 0) {
            zero_packets_in_os_queue_cnt_B++;
        } 
        #ifndef UDP_HIL_RESPONSE_TYPE_ALWAYS
            else {
                while(1) {
                    printf("UDP_HIL: pending_bytes: %u, but UDP_HIL_RESPONSE_TYPE_ALWAYS is not set, so this should not happen!!!\n", pending_bytes);
                }
            }
        #endif
        else if (pending_bytes == sizeof(DataStruct)) {
            one_packets_in_os_queue_cnt_B++;
            #ifdef UDP_HIL_CLEAR_OS_UDP_QUEUE
                recvfrom(udp_hil_socket, &buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
                addInSeq();
            #endif
        } else {
            more_packets_in_os_queue_cnt_B++;
        }
    }

}

#endif // UDP_HIL_ENABLED