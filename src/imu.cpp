#include "imu.hpp"

#define MPU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_OUT    0x3B

static const float G_CONST = 9.81f;
static const float D2R = 0.0174533f, R2D = 57.29578f;

// ---- 3x3 矩阵 (列主序 M[i + 3*j]) ----
static void m3Identity(float* m)
{
    m[0] = 1; m[1] = 0; m[2] = 0;
    m[3] = 0; m[4] = 1; m[5] = 0;
    m[6] = 0; m[7] = 0; m[8] = 1;
}
static void m3DiagAdd(float* m, float d)
{
    m[0] += d; m[4] += d; m[8] += d;
}
static void m3Mul(float* c, const float* a, const float* b)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float s = 0;
            for (int k = 0; k < 3; k++)
            {
                s += a[i + 3 * k] * b[k + 3 * j];
            }
            c[i + 3 * j] = s;
        }
    }
}
static void m3MulT(float* c, const float* a, const float* b)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float s = 0;
            for (int k = 0; k < 3; k++)
            {
                s += a[i + 3 * k] * b[j + 3 * k];
            }
            c[i + 3 * j] = s;
        }
    }
}
static bool m3Inv(float* inv, const float* m)
{
    float d = m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[3] * (m[1] * m[8] - m[2] * m[7])
            + m[6] * (m[1] * m[5] - m[2] * m[4]);
    if (fabs(d) < 1e-9f)
    {
        return false;
    }
    float id = 1.0f / d;
    inv[0] = (m[4] * m[8] - m[5] * m[7]) * id;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * id;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * id;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * id;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * id;
    inv[5] = (m[2] * m[3] - m[0] * m[5]) * id;
    inv[6] = (m[3] * m[7] - m[4] * m[6]) * id;
    inv[7] = (m[1] * m[6] - m[0] * m[7]) * id;
    inv[8] = (m[0] * m[4] - m[1] * m[3]) * id;
    return true;
}
static void m3Skew(float* s, float x, float y, float z)
{
    s[0] = 0;  s[3] = -z; s[6] = y;
    s[1] = z;  s[4] = 0;  s[7] = -x;
    s[2] = -y; s[5] = x;  s[8] = 0;
}

// ---- 四元数 ----
static void quatNorm(float* q)
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1e-9f)
    {
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    }
}
static void quatMul(float* r, const float* a, const float* b)
{
    r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}
static void quatFromRot(float* q, float x, float y, float z)
{
    float a = sqrtf(x * x + y * y + z * z);
    if (a < 1e-9f)
    {
        q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
        return;
    }
    float ha = a * 0.5f, s = sinf(ha) / a;
    q[0] = cosf(ha);
    q[1] = x * s; q[2] = y * s; q[3] = z * s;
}
static void quatToEuler(float& roll, float& pitch, float& yaw, const float* q)
{
    float w = q[0], x = q[1], y2 = q[2], z = q[3];
    roll  = atan2f(2.0f * (w * x + y2 * z), 1.0f - 2.0f * (x * x + y2 * y2)) * R2D;
    float sp = 2.0f * (w * y2 - z * x);
    pitch = (fabs(sp) >= 1) ? copysignf(90.0f, sp) : asinf(sp) * R2D;
    yaw   = atan2f(2.0f * (w * z + x * y2), 1.0f - 2.0f * (y2 * y2 + z * z)) * R2D;
}
static void quatPredGrav(float* h, const float* q)
{
    float w = q[0], x = q[1], y2 = q[2], z = q[3];
    h[0] = 2.0f * (x * z - w * y2);
    h[1] = 2.0f * (y2 * z + w * x);
    h[2] = w * w - x * x - y2 * y2 + z * z;
}

// ============================================================
Mpu6050Ekf::Mpu6050Ekf(const ImuConfig& cfg) : config_(cfg)
{
    memset(&state_, 0, sizeof(state_));
    state_.q0 = 1;
}

bool Mpu6050Ekf::begin()
{
    Wire.begin(config_.sda, config_.scl);
    Wire.setClock(400000);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    calibrateBias(100);
    reset();
    state_.ok = true;
    return true;
}

void Mpu6050Ekf::reset()
{
    quat_[0] = 1; quat_[1] = 0; quat_[2] = 0; quat_[3] = 0;
    m3Identity(cov_);
    prev_w_[0] = prev_w_[1] = prev_w_[2] = 0;
    state_.yaw = 0;
}

void Mpu6050Ekf::readRaw_(int16_t& ax, int16_t& ay, int16_t& az,
                           int16_t& gx, int16_t& gy, int16_t& gz)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14);
    if (Wire.available() < 14)
    {
        ax = ay = az = gx = gy = gz = 0;
        return;
    }
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

void Mpu6050Ekf::calibrateBias(int samples)
{
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < samples; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6);
        if (Wire.available() >= 6)
        {
            int16_t gx = Wire.read() << 8 | Wire.read();
            int16_t gy = Wire.read() << 8 | Wire.read();
            int16_t gz = Wire.read() << 8 | Wire.read();
            sx += gx; sy += gy; sz += gz;
        }
        delay(2);
    }
    state_.bias_x = sx / samples * config_.gyro_scale * D2R;
    state_.bias_y = sy / samples * config_.gyro_scale * D2R;
    state_.bias_z = sz / samples * config_.gyro_scale * D2R;
}

void Mpu6050Ekf::ekfStep_(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt)
{
    float dq[4];
    quatFromRot(dq, gx * dt, gy * dt, gz * dt);
    float qp[4];
    quatMul(qp, quat_, dq);

    float f[9];
    m3Identity(f);
    {
        float s[9];
        m3Skew(s, gx, gy, gz);
        for (int i = 0; i < 9; i++)
        {
            f[i] -= s[i] * dt;
        }
    }

    float qd[9] = {};
    m3DiagAdd(qd, config_.ekf_q * dt * dt);
    float t1[9], pp[9];
    m3Mul(t1, f, cov_);
    m3MulT(pp, t1, f);
    for (int i = 0; i < 9; i++)
    {
        pp[i] += qd[i];
    }

    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 1.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            quat_[i] = qp[i];
        }
        for (int i = 0; i < 9; i++)
        {
            cov_[i] = pp[i];
        }
        return;
    }
    float z[3] = {ax / an, ay / an, az / an};
    float h[3];
    quatPredGrav(h, qp);

    float H[9];
    m3Skew(H, -h[0], -h[1], -h[2]);
    float y[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]};

    float hp[9], S[9];
    m3Mul(hp, H, pp);
    m3MulT(S, hp, H);
    m3DiagAdd(S, config_.ekf_r);

    float pht[9], si[9];
    m3MulT(pht, pp, H);
    if (!m3Inv(si, S))
    {
        for (int i = 0; i < 4; i++)
        {
            quat_[i] = qp[i];
        }
        for (int i = 0; i < 9; i++)
        {
            cov_[i] = pp[i];
        }
        return;
    }
    float K[9];
    m3Mul(K, pht, si);

    float dt3[3] = {};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            dt3[i] += K[i + 3 * j] * y[j];
        }
    }

    float qc[4];
    quatFromRot(qc, dt3[0], dt3[1], dt3[2]);
    quatMul(quat_, qp, qc);
    quatNorm(quat_);

    float kh[9];
    m3Mul(kh, K, H);
    float ikh[9];
    for (int i = 0; i < 9; i++)
    {
        ikh[i] = (i % 4 == 0 ? 1.0f : 0.0f) - kh[i];
    }
    m3Mul(cov_, ikh, pp);
}

void Mpu6050Ekf::update(float dt)
{
    if (!state_.ok)
    {
        return;
    }

    int16_t rax, ray, raz, rgx, rgy, rgz;
    readRaw_(rax, ray, raz, rgx, rgy, rgz);

    state_.accel_x = rax * config_.accel_scale * G_CONST * config_.acc_sgn[0];
    state_.accel_y = ray * config_.accel_scale * G_CONST * config_.acc_sgn[1];
    state_.accel_z = raz * config_.accel_scale * G_CONST * config_.acc_sgn[2];

    float wx = rgx * config_.gyro_scale * D2R * config_.gyr_sgn[0] - state_.bias_x;
    float wy = rgy * config_.gyro_scale * D2R * config_.gyr_sgn[1] - state_.bias_y;
    float wz = rgz * config_.gyro_scale * D2R * config_.gyr_sgn[2] - state_.bias_z;

    // IMU → 车体 3D 旋转
    float ry = config_.rot_yaw * D2R, rp = config_.rot_pitch * D2R, rr = config_.rot_roll * D2R;
    float cy = cosf(ry), sy = sinf(ry), cp = cosf(rp), sp = sinf(rp), cr = cosf(rr), sr = sinf(rr);
    float R00 = cy * cp,        R10 = sy * cp,        R20 = -sp;
    float R01 = cy * sp * sr - sy * cr, R11 = sy * sp * sr + cy * cr, R21 = cp * sr;
    float R02 = cy * sp * cr + sy * sr, R12 = sy * sp * cr - cy * sr, R22 = cp * cr;

    float ar = R00 * state_.accel_x + R01 * state_.accel_y + R02 * state_.accel_z;
    float a2 = R10 * state_.accel_x + R11 * state_.accel_y + R12 * state_.accel_z;
    float a3 = R20 * state_.accel_x + R21 * state_.accel_y + R22 * state_.accel_z;
    state_.accel_x = ar; state_.accel_y = a2; state_.accel_z = a3;

    float w1 = wx, w2 = wy, w3 = wz;
    wx = R00 * w1 + R01 * w2 + R02 * w3;
    wy = R10 * w1 + R11 * w2 + R12 * w3;
    wz = R20 * w1 + R21 * w2 + R22 * w3;

    // 离心 + 切向补偿
    float dax = wx - prev_w_[0], day = wy - prev_w_[1], daz = wz - prev_w_[2];
    float ax_dt = dax / dt, ay_dt = day / dt, az_dt = daz / dt;
    prev_w_[0] = wx; prev_w_[1] = wy; prev_w_[2] = wz;

    float ox = config_.off[0], oy = config_.off[1], oz = config_.off[2];
    float wdr = wx * ox + wy * oy + wz * oz;
    float wm2 = wx * wx + wy * wy + wz * wz;

    state_.accel_x -= (wx * wdr - ox * wm2);
    state_.accel_y -= (wy * wdr - oy * wm2);
    state_.accel_z -= (wz * wdr - oz * wm2);
    state_.accel_x -= (ay_dt * oz - az_dt * oy);
    state_.accel_y -= (az_dt * ox - ax_dt * oz);
    state_.accel_z -= (ax_dt * oy - ay_dt * ox);

    state_.gyro_x = wx; state_.gyro_y = wy; state_.gyro_z = wz;

    ekfStep_(wx, wy, wz, state_.accel_x, state_.accel_y, state_.accel_z, dt);
    quatToEuler(state_.roll, state_.pitch, state_.yaw, quat_);
    state_.q0 = quat_[0]; state_.q1 = quat_[1]; state_.q2 = quat_[2]; state_.q3 = quat_[3];
    state_.fallen = (sqrtf(state_.roll * state_.roll + state_.pitch * state_.pitch) > config_.fall_tilt);
}
