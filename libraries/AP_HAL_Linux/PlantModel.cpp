#include "PlantModel.h"
#include <stdio.h>

void PlantModel::init()
{
    position.zero();
    velocity.zero();
    omega.zero();
    accel.zero();
    gyro.zero();

    q_att = Quaternion(); // Einheits‑Quat

    inertia = Matrix3f();
    inertia.a.x = PLANT_INERTIA_ROLL;
    inertia.b.y = PLANT_INERTIA_PITCH;
    inertia.c.z = PLANT_INERTIA_YAW;

    for (float &m : motor_outputs) m = 0.0f;

    on_gnd = true;
}

//--------------------------------------------------

void PlantModel::update(float dt)
{
    // 1) Rotationsmatrix
    Matrix3f R;
    q_att.rotation_matrix(R);

    // 2) Kräfte
    Vector3f F_thrust = R.transposed() * thrust_body();
    Vector3f F_g(0,0, -GRAVITY_MSS * PLANT_MASS);
    Vector3f F_total = F_thrust + F_g;

    if (fabsf(F_thrust.z) > fabsf(F_g.z)) {
        on_gnd = false;
    } else if (motor_outputs[0]==0 && motor_outputs[1]==0 &&
               motor_outputs[2]==0 && motor_outputs[3]==0) {
        on_gnd = true;
    }

    if (on_gnd) {
        F_total.zero();
        velocity.zero();
        position.zero();
        omega.zero();
    }
    velocity.zero();
    position.zero();

    // 3) Translation
    Vector3f acc_world = F_total / PLANT_MASS;
    velocity += acc_world * dt;
    position += velocity * dt;

    // 4) Drehmomente
    Vector3f torque = torque_body();
    Vector3f I_diag(inertia.a.x, inertia.b.y, inertia.c.z);
    Vector3f omega_cross_Iomega = omega % Vector3f(I_diag.x*omega.x,
                                                   I_diag.y*omega.y,
                                                   I_diag.z*omega.z);
    Vector3f omega_dot;
    omega_dot.x = (torque.x - omega_cross_Iomega.x) / I_diag.x;
    omega_dot.y = (torque.y - omega_cross_Iomega.y) / I_diag.y;
    omega_dot.z = (torque.z - omega_cross_Iomega.z) / I_diag.z;
    omega += omega_dot * dt;
    gyro = omega; // IMU‑Gyro

    // 5) Quaternion‑Integration
    Quaternion q_rate(0, 0.5f*omega.x, 0.5f*omega.y, 0.5f*omega.z);
    q_rate = q_rate * Quaternion(dt, 0, 0, 0);
    q_att = q_att * q_rate;
    q_att.normalize();

    // 6) IMU‑Beschleunigung
    accel = R.transposed() * (Vector3f(0,0, -GRAVITY_MSS) + acc_world);

    Vector3f angles = getEuler();
    static int count = 0;
    count++;
    if((!on_gnd||1)&&count % 100 == 0) {
        printf("ong: %i\n", on_gnd);
        printf("wld: %f, %f, %f\n", F_thrust.x, F_thrust.y, F_thrust.z);
        printf("bdy: %f, %f, %f\n", accel.x, accel.y, accel.z);
        //printf("acc: %f, %f, %f\n", thrust_body().x, thrust_body().y, thrust_body().z);
        printf("angles: %f, %f, %f\n", degrees(angles.x), degrees(angles.y), degrees(angles.z));
        printf("position: %f, %f, %f\n", position.x, position.y, position.z);
    }
}

//--------------------------------------------------

Vector3f PlantModel::thrust_body()
{
    float total = 0.0f;
    for (float m : motor_outputs) total += constrain_float(m,0.0f,1.0f);
    return Vector3f(0,0, total * PLANT_THRUST_FACT); // +z nach unten
}

//--------------------------------------------------

Vector3f PlantModel::torque_body()
{
    float L = PLANT_ARM_LENGTH;
    float m[4];
    for (int i=0;i<4;i++) m[i] = constrain_float(motor_outputs[i],0.0f,1.0f);

    float tau_x =  (L/sqrtf(2.0f))*PLANT_THRUST_FACT*(-m[0]-m[3]+m[1]+m[2]);
    float tau_y =  (L/sqrtf(2.0f))*PLANT_THRUST_FACT*( m[0]+m[2]-m[1]-m[3]);
    float tau_z =  PLANT_YAW_TORQUE_FACT*PLANT_THRUST_FACT*(m[0]+m[1]-m[2]-m[3]);

    return Vector3f(tau_x, tau_y, tau_z);
}

//--------------------------------------------------

Matrix3f PlantModel::rotation_matrix() { 
    Matrix3f M;
    q_att.rotation_matrix(M);
    return M;
}

Vector3f PlantModel::getAccel() { return accel; }
Vector3f PlantModel::getGyro()  { return gyro;  }

void PlantModel::setMotor(uint8_t ch, uint16_t pwm)
{
    if (ch < 4) motor_outputs[ch] = (pwm - 1000) / 950.0f;
}
