#pragma once

// config for UDP_HIL
//#define UDP_HIL_ENABLED // activates scheduler call to udp_hil
//#define UDP_HIL_MS5611   // activates ms5611 (baro) data replacement from udp_hil
//#define UDP_HIL_MPU9250  // activates mpu9250 (imu, mag) data replacement from udp_hil
//#define UDP_HIL_UBLOX     // activates ublox (gps) data replacement from udp_hil
//#define UDP_HIL_PWM       // activates pwm (motor) data grabbing to udp_hil




// Experiments
// only one per class at a time !!!

//#define TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION

//same but different clocks
//#define TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM
//#define TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_PROCESS
//#define TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_THREAD

//#define TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM
//#define TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_PROCESS
//#define TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_THREAD

#define TIMING_EXPERIMENT_MS5611_CONVERSION_CALL_PRECISION

//#define TIMING_EXPERIMENT_MS5611_CONVERSION_DURATION_SYSTEM
//#define TIMING_EXPERIMENT_MS5611_CONVERSION_DURATION_PROCESS
//#define TIMING_EXPERIMENT_MS5611_CONVERSION_DURATION_THREAD

#define TIMING_EXPERIMENT_MPU9250_POLLDATA_CALL_PRECISION

//#define TIMING_EXPERIMENT_MPU9250_POLLDATA_DURATION_SYSTEM
//#define TIMING_EXPERIMENT_MPU9250_POLLDATA_DURATION_PROCESS
//#define TIMING_EXPERIMENT_MPU9250_POLLDATA_DURATION_THREAD


// Experiment output
// only one per class at a time !!!

//#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) printf("UDP_HIL_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_udp_hil_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
/*
#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) do { \
    static const int buffer_size = 10; \
    static long long buffer[buffer_size]; \
    static int buffer_index = 0; \
    buffer[buffer_index++] = x; \
    if (buffer_index >= buffer_size) { \
        FILE *file = fopen("/home/pi/experiment_udp_hil_cum_output.txt", "a"); \
        if (file) { \
            char temp_output[1024]; \
            int offset = 0; \
            offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "STARTING_FILE_BLOCK_WRITE\n",); \
            for (int i = 0; i < buffer_size; i++) { \
                offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "%lli\n", buffer[i]); \
                if (offset >= sizeof(temp_output)) break; \
            } \
            fprintf(file, "%s", temp_output); \
            fclose(file); \
        } \
        buffer_index = 0; \
    } \
} while (0)
*/

//#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) printf("MS5611_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_MS5611_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
/*
#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) do { \
    static const int buffer_size = 10; \
    static long long buffer[buffer_size]; \
    static int buffer_index = 0; \
    buffer[buffer_index++] = x; \
    if (buffer_index >= buffer_size) { \
        FILE *file = fopen("/home/pi/experiment_MS5611_cum_output.txt", "a"); \
        if (file) { \
            char temp_output[1024]; \
            int offset = 0; \
            offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "STARTING_FILE_BLOCK_WRITE\n",); \
            for (int i = 0; i < buffer_size; i++) { \
                offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "%lli\n", buffer[i]); \
                if (offset >= sizeof(temp_output)) break; \
            } \
            fprintf(file, "%s", temp_output); \
            fclose(file); \
        } \
        buffer_index = 0; \
    } \
} while (0)
*/


//#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) printf("MPU9250_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_MPU9250_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
/*
#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) do { \
    static const int buffer_size = 10; \
    static long long buffer[buffer_size]; \
    static int buffer_index = 0; \
    buffer[buffer_index++] = x; \
    if (buffer_index >= buffer_size) { \
        FILE *file = fopen("/home/pi/experiment_MPU9250_cum_output.txt", "a"); \
        if (file) { \
            char temp_output[1024]; \
            int offset = 0; \
            offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "STARTING_FILE_BLOCK_WRITE\n",); \
            for (int i = 0; i < buffer_size; i++) { \
                offset += snprintf(temp_output + offset, sizeof(temp_output) - offset, \
                                   "%lli\n", buffer[i]); \
                if (offset >= sizeof(temp_output)) break; \
            } \
            fprintf(file, "%s", temp_output); \
            fclose(file); \
        } \
        buffer_index = 0; \
    } \
} while (0)
*/