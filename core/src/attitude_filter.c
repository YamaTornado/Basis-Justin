#include "fc/attitude_filter.h"

#include <math.h>
#include <stddef.h>

static float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

void fc_attitude_filter_init(fc_attitude_filter_t *f, float beta, int have_mag) {
    f->q[0] = 1.0f;
    f->q[1] = 0.0f;
    f->q[2] = 0.0f;
    f->q[3] = 0.0f;
    f->beta = beta;
    f->have_mag = have_mag;
}

/* Standard Madgwick AHRS update (gradient descent algorithm, Madgwick 2010),
 * with a MARG (mag) variant when a magnetometer sample is supplied and an
 * IMU-only (gyro+accel) variant otherwise. Gyro input in rad/s internally;
 * accel/mag do not need to be normalized on input, they are normalized here. */
void fc_attitude_filter_update(fc_attitude_filter_t *f, const float gyro_dps[3],
                                const float accel_g[3], const float mag_ut[3], float dt_s) {
    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];

    float gx = gyro_dps[0] * (float)M_PI / 180.0f;
    float gy = gyro_dps[1] * (float)M_PI / 180.0f;
    float gz = gyro_dps[2] * (float)M_PI / 180.0f;

    float ax = accel_g[0], ay = accel_g[1], az = accel_g[2];

    /* Rate of change of quaternion from gyroscope */
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    int use_correction = norm > 1e-6f;

    if (use_correction) {
        float recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        float s0, s1, s2, s3;

        if (f->have_mag && mag_ut != NULL) {
            float mx = mag_ut[0], my = mag_ut[1], mz = mag_ut[2];
            float mag_norm_sq = mx * mx + my * my + mz * mz;
            if (mag_norm_sq > 1e-6f) {
                recip_norm = inv_sqrt(mag_norm_sq);
                mx *= recip_norm;
                my *= recip_norm;
                mz *= recip_norm;

                /* Auxiliary variables to avoid repeated arithmetic */
                float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz;
                float _2q1mx = 2.0f * q1 * mx;
                float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
                float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
                float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
                float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

                float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
                           _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
                float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
                           my * q2q2 + _2q2 * mz * q3 - my * q3q3;
                float _2bx = sqrtf(hx * hx + hy * hy);
                float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
                             _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
                float _4bx = 2.0f * _2bx;
                float _4bz = 2.0f * _2bz;

                s0 = -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay) -
                     _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                     (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                     _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s1 = _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) -
                     4.0f * q1 * (1.0f - 2.0f * (q1q1 + q2q2) - az) +
                     _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                     (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                     (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s2 = -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) -
                     4.0f * q2 * (1.0f - 2.0f * (q1q1 + q2q2) - az) +
                     (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                     (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                     (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s3 = _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay) +
                     (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                     (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                     _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            } else {
                use_correction = 0;
                s0 = s1 = s2 = s3 = 0.0f;
            }
        } else {
            /* IMU-only gradient descent (accel only) */
            float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
            float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
                 _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
                 _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        }

        if (use_correction) {
            float norm_s = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (norm_s > 1e-6f) {
                float recip = 1.0f / norm_s;
                s0 *= recip;
                s1 *= recip;
                s2 *= recip;
                s3 *= recip;

                qDot1 -= f->beta * s0;
                qDot2 -= f->beta * s1;
                qDot3 -= f->beta * s2;
                qDot4 -= f->beta * s3;
            }
        }
    }

    q0 += qDot1 * dt_s;
    q1 += qDot2 * dt_s;
    q2 += qDot3 * dt_s;
    q3 += qDot4 * dt_s;

    float recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    f->q[0] = q0 * recip_norm;
    f->q[1] = q1 * recip_norm;
    f->q[2] = q2 * recip_norm;
    f->q[3] = q3 * recip_norm;
}

void fc_attitude_filter_rotate_to_world(const fc_attitude_filter_t *f, const float body[3],
                                         float world_out[3]) {
    float w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];

    /* v_world = q * v_body * q_conjugate, expanded */
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    world_out[0] = (1.0f - 2.0f * (yy + zz)) * body[0] + 2.0f * (xy - wz) * body[1] +
                   2.0f * (xz + wy) * body[2];
    world_out[1] = 2.0f * (xy + wz) * body[0] + (1.0f - 2.0f * (xx + zz)) * body[1] +
                   2.0f * (yz - wx) * body[2];
    world_out[2] = 2.0f * (xz - wy) * body[0] + 2.0f * (yz + wx) * body[1] +
                   (1.0f - 2.0f * (xx + yy)) * body[2];
}
