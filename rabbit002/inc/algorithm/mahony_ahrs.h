#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

class MahonyAHRS {
public:
    MahonyAHRS(double twoKp = 2.0f, double twoKi = 0, double q0 = 0.95f, double q1 = 0.0f, double q2 = 0.0f, double q3 = 0.30f);
    ~MahonyAHRS();

    void updateIMU(double gx, double gy, double gz,
                   double ax, double ay, double az,
                   double dt, int mode);

    double q0, q1, q2, q3;
    double roll, pitch, yaw;

private:
    double twoKp;
    double twoKi;
    double integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
};

#endif
