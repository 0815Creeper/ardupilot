#pragma once

// configuration
#define UDP_HIL_FREQ    1000 // frequency of udp_hil data exchange
#define UDP_HIL_RESET_AFTER_MILLIS 10000 // reset after this many millis
#define UDP_HIL_RESPONSE_TYPE_ALWAYS // vs only on recive
#ifdef UDP_HIL_RESPONSE_TYPE_ALWAYS
//#define UDP_HIL_CLEAR_OS_UDP_QUEUE // only available if UDP_HIL_RESPONSE_TYPE_ALWAYS is set, will increase package loss (by skips) but reduce latency
#endif
#define UDP_HIL_RESPONSE_TYPE_ALWAYS_START_AFTER (3*UDP_HIL_FREQ) // ticks after init to start sending responses in "always" mode. Before, we only send on recive
#define UDP_HIL_DEBUG_PACKET_STATS_INTERVAL (10*UDP_HIL_FREQ) // how often to print packet stats, 10 seconds in this case 
#define UDP_HIL_DEBUG_PACKET_TIMOUTS_AND_SKIPS
#define UDP_HIL_DEBUG_PACKET_STATS_INTERVAL_PRINT
//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_RECV_PORT 13018
//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_SEND_PORT 13019
//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_IP "10.42.0.1"

#define GET_TIME_NS(x) ((uint64_t)x.tv_sec * (uint64_t)1000000000UL + (uint64_t)x.tv_nsec)
// end of configuration


// toggle options
#define UDP_HIL_ENABLED // activates scheduler call to udp_hil
#ifdef UDP_HIL_ENABLED
#define UDP_HIL_MS5611   // activates ms5611 (baro) data replacement from udp_hil
#define UDP_HIL_MPU9250  // activates mpu9250 (imu) data replacement from udp_hil
#define UDP_HIL_AK8963  // activates mpu9250 (mag) data replacement from udp_hil
#define UDP_HIL_UBLOX     // activates ublox (gps) data replacement from udp_hil
#define UDP_HIL_PWM       // activates pwm (motor) data grabbing to udp_hil
#endif

//#define UDP_HIL_DISABLE_SEQ_CHECK // disables sequence number checking in udp_hil, use with care, only for testing

//#define FIX_MPU9250_FIFO_LEN_RESET
//#define DISABLE_INTERNAL_ERROR_IMU_RESET
//#define MPU9250_FORCE_LOWER_RATE 250 // 1kHz meassurements (fifo) and set communication rate to value

//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_SEND_TO
//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_RECV
//#define TIMING_EXPERIMENT_MPU9250_UDP_DECENTRALIZED_RECV_FROM

//#define SELECTIVE_JITTER_INJETION_MS5611_ENABLED // enables selective jitter injection for ms5611 on main thread

//end of toggle options

#ifdef SELECTIVE_JITTER_INJETION_MS5611_ENABLED
#define SELECTIVE_JITTER_INJETION_MS5611_DURATION 20000// fails with crash 100000 // in microseconds per loop
#define SELECTIVE_JITTER_INJETION_MS5611_GX_THRESHOLD 10
#endif

// Experiments
// only one per class at a time (except ublox) !!!

//#define TIMING_EXPERIMENT_UDP_HIL_TIMER_TICK_CALL_PRECISION

//#define TIMING_EXPERIMENT_UDP_HIL_RECVFROM_DURATION_SYSTEM

//#define TIMING_EXPERIMENT_UDP_HIL_SENDTO_DURATION_SYSTEM

//#define TIMING_EXPERIMENT_UDP_HIL_SENDTO_DONE_PRECISION

//#define TIMING_EXPERIMENT_UDP_HIL_PINGPONG_DURATION_SYSTEM

#define TIMING_EXPERIMENT_MS5611_CONVERSION_CALL_PRECISION

//#define TIMING_EXPERIMENT_MS5611_CONVERSION_DURATION_SYSTEM

#define TIMING_EXPERIMENT_MPU9250_POLLDATA_CALL_PRECISION

//#define TIMING_EXPERIMENT_MPU9250_POLLDATA_DURATION_SYSTEM

//#define TIMING_EXPERIMENT_AK8963_UPDATE_CALL_PRECISION

//#define TIMING_EXPERIMENT_UBLOX_PARSE_GPS_CALL_PRECISION

//#define TIMING_EXPERIMENT_UBLOX_PARSE_GPS_CALL_MSG_PRECISION

//#define TIMING_EXPERIMENT_PWM_SET_DUTY_CYCLE_CALL_PRECISION
//end of Experiments


// Experiment output
// only one of same name at a time !!!

//#define TIMING_EXPERIMENT_UDP_HIL_GPIO_OUTPUT_26_ENABLED
#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) printf("UDP_HIL_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_udp_hil_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_UDP_HIL_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_udp_hil_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)

#define TIMING_SENDTO_DONE_PRECISION_UDP_HIL_OUTPUT(x) printf("UDP_HIL_TIMING_SENDTO_DONE_PRECISION_OUTPUT: %lli\n", x)

#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) printf("MS5611_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_MS5611_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_MS5611_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_MS5611_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)

#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) printf("MPU9250_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_MPU9250_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_MPU9250_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_MPU9250_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)


#define TIMING_EXPERIMENT_AK8963_OUTPUT(x) printf("AK8963_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_AK8963_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_AK8963_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_AK8963_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_AK8963_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)

#define TIMING_EXPERIMENT_UBLOX_OUTPUT(x) printf("UBLOX_TIMING_EXPERIMENT_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_UBLOX_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_UBLOX_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_UBLOX_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_UBLOX_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)

#define TIMING_EXPERIMENT_UBLOX_CALLPRECMSG_OUTPUT(x) printf("UBLOX_TIMING_EXPERIMENT_CALLPRECMSG_OUTPUT: %lli\n", x)
//#define TIMING_EXPERIMENT_UBLOX_OUTPUT(x) do { FILE *file = fopen("/home/pi/experiment_UBLOX_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_UBLOX_OUTPUT(x) do { FILE *file = fopen("/run/testfiles/experiment_UBLOX_output.txt", "a"); if (file) { fprintf(file, "%lli\n", x); fclose(file); } } while (0)

#define TIMING_EXPERIMENT_PWM_OUTPUT(c,x) printf("PWM%hhu_TIMING_EXPERIMENT_OUTPUT: %lli\n", c, x)
//#define TIMING_EXPERIMENT_PWM_OUTPUT(c,x) do { FILE *file = fopen("/home/pi/experiment_PWM_output.txt", "a"); if (file) { fprintf(file, "%hhu: %lli\n", c, x); fclose(file); } } while (0)
//#define TIMING_EXPERIMENT_PWM_OUTPUT(c,x) do { FILE *file = fopen("/run/testfiles/experiment_PWM_output.txt", "a"); if (file) { fprintf(file, "%hhu: %lli\n", c, x); fclose(file); } } while (0)

#define SELECTIVE_JITTER_INJETION_MS5611_OUTPUT(x) printf("SELECTIVE_JITTER_INJETION_MS5611_OUTPUT: %lli\n", (long long int)x)


// end of Experiment output