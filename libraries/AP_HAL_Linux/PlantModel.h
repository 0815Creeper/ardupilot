#pragma once

#include <AP_Math/AP_Math.h>

// Plant Parameters
#define PLANT_INERTIA_ROLL  1.43e-3f // Roll inertia
#define PLANT_INERTIA_PITCH 1.43e-3f // Pitch inertia
#define PLANT_INERTIA_YAW   2.89e-3f // Yaw inertia
#define PLANT_MASS            0.622f // kg
#define PLANT_ARM_LENGTH      0.185f // m (370 mm Diagonale)
#define PLANT_THRUST_FACT      5.00f // thrust factor (N)
#define PLANT_YAW_TORQUE_FACT  0.05f // Yaw torque factor

#define LAT_SCALE (10000000.0/111120.0) // 1e7deg/m
#define LON_SCALE (10000000.0/74625.0) // 1e7deg/m; at 48°latitude

class PlantModel {
public:
    void init();
    void update(float dt);
    
    //void getIMUsamples(uint16_t* buff, int16_t t2, uint8_t num_samples);
    Vector3f getAccel();
    Vector3f getGyro();
    bool isOnGround() { return on_gnd; }
    void setMotor(uint8_t ch, uint16_t pwm);
    Vector3f getPosition() { return position; }
    Vector3f getVelocity() { return velocity; }
    void addGPS(int32_t* lat, int32_t* lon, int32_t* alt);
    void setBaro(float* pressure);
    bool on_gnd = true;

private:
    // Sensor-Ausgaben (wie IMU)
    Vector3f accel;   // m/s²
    Vector3f gyro;    // rad/s

    // Zustand (optional zur Analyse)
    Vector3f position;   // m
    Vector3f velocity;   // m/s
    Vector3f angles;     // rad (Roll, Pitch, Yaw)
    Vector3f omega;      // rad/s

    // Motorbefehle [0 … +1]
    float motor_outputs[4]; // 4 Motoren in X-Konfiguration

    float mass = PLANT_MASS;
    float arm_length = PLANT_ARM_LENGTH;
    Matrix3f inertia;


    // Intern
    Vector3f thrust_body();
    Vector3f torque_body();
    Matrix3f rotation_matrix();
    
};
