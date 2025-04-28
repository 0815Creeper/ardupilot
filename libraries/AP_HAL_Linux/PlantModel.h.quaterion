#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/quaternion.h>

// Plant Parameters
#define PLANT_INERTIA_ROLL  0.03//1.43e-3f // Roll inertia
#define PLANT_INERTIA_PITCH 0.03//1.43e-3f // Pitch inertia
#define PLANT_INERTIA_YAW   0.06//2.89e-3f // Yaw inertia
#define PLANT_MASS            1.0f //0.622f // kg
#define PLANT_ARM_LENGTH      0.185f // m (370 mm Diagonale)
#define PLANT_THRUST_FACT      8.00f // thrust factor (N)
#define PLANT_YAW_TORQUE_FACT  0.05f // Yaw torque factor

class PlantModel {
public:
    // --- Lifecycle ------------------------------------------------
    void init();                // Zustand auf Null / Hover-Reset
    void update(float dt);      // Hauptaufruf (dt in Sekunden, z.B. 0.001f)

    // --- IMU-Schnittstelle ---------------------------------------
    Vector3f getAccel();        // m/s^2, IMU-Frame (Body)
    Vector3f getGyro();         // rad/s  , IMU-Frame (Body)

    // --- Hilfsfunktionen -----------------------------------------
    Matrix3f rotation_matrix(); // Body -> Welt Rotmat (falls extern gebraucht)
    Vector3f getEuler() const { 
        Vector3f v;
        q_att.to_euler(v);
        return v;
    } // Roll, Pitch, Yaw (rad)

    void setMotor(uint8_t ch, uint16_t pwm); // PWM 1000–1950 -> Motor 0–1

    // --- Öffentliche Felder --------------------------------------
    Vector3f position;   // m    (Welt)
    Vector3f velocity;   // m/s  (Welt)

    bool on_gnd = true;  // einfacher Bodenstatus

private:
    // --- Zustand --------------------------------------------------
    Quaternion q_att;     // Orientierung (Body -> Welt)
    Vector3f   omega;     // rad/s  (Body)
    Vector3f   accel;     // m/s^2  (Body)
    Vector3f   gyro;      // rad/s  (Body)

    // --- Parameter / Motoren -------------------------------------
    float motor_outputs[4]; // 0..1 normiert

    // --- Inertia --------------------------------------------------
    Matrix3f inertia;     // diagonal (Ix,Iy,Iz)

    // --- Interne Hilfsfunktionen ---------------------------------
    Vector3f thrust_body();   // Schub-Vektor im Body frame
    Vector3f torque_body();   // Roll/Pitch/Yaw-Momente im Body frame
};
