#include "mahony_ahrs.h"

// Yaw(y), pitch(z), roll(x)
#include <cmath>

MahonyAHRS::MahonyAHRS(double kp, double ki, double q0, double q1, double q2, double q3)
    : twoKp(kp), twoKi(ki), q0(q0), q1(q1), q2(q2), q3(q3) {}

MahonyAHRS::~MahonyAHRS() {}

void MahonyAHRS::updateIMU(double gx, double gy, double gz,
                           double ax, double ay, double az,
                           double dt, int mode) {

    double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 15.0f) {
        return;
    } else if (norm > 7.0f && mode == 0) {
        ax /= norm;
        ay /= norm;
        az /= norm;

        double halfvx = q1 * q2 - q0 * q3;
        double halfvy = q0 * q0 + q2 * q2 - 0.5f;
        double halfvz = q0 * q1 + q2 * q3;

        double halfex = (ay * halfvz - az * halfvy);
        double halfey = (az * halfvx - ax * halfvz);
        double halfez = (ax * halfvy - ay * halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;

            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    } else if (mode == 1) {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    double qDot1 = -q1 * gx - q2 * gy - q3 * gz;
    double qDot2 = q0 * gx + q2 * gz - q3 * gy;
    double qDot3 = q0 * gy - q1 * gz + q3 * gx;
    double qDot4 = q0 * gz + q1 * gy - q2 * gx;

    q0 += qDot1;
    q1 += qDot2;
    q2 += qDot3;
    q3 += qDot4;

    norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    yaw = std::atan2(-2.0f * (q0 * q2 + q1 * q3), -1.0f + 2.0f * (q2 * q2 + q3 * q3));
    pitch = std::asin(-2.0f * (q0 * q3 - q1 * q2));
    roll = std::atan2(-2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q3 * q3));
}
