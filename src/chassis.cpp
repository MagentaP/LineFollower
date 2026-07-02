#include "chassis.hpp"

static const float D2R = 0.0174533f, R2D = 57.29578f;

Chassis::Chassis(Motor& l, Motor& r)
    : left_(l), right_(r),
      pid_yaw_(yaw_kp, yaw_ki, yaw_kd,
               -max_angular_velocity, max_angular_velocity, millis(), true)
{
    imu_.begin();
    time_ms_ = millis();
}

// ============================================================
// 公共接口
// ============================================================
void Chassis::setVelocity(float v, float w)
{
    if (pid_enabled_ && left_.has_encoder_ && right_.has_encoder_)
    {
        float vl = v - w * wheel_dist / 2.0f;
        float vr = v + w * wheel_dist / 2.0f;
        left_.setTargetRadS(vl / wheel_radius);
        right_.setTargetRadS(vr / wheel_radius);
        return;
    }

    float vl = v - w * wheel_dist / 2.0f;
    float vr = v + w * wheel_dist / 2.0f;
    float dl = (vl / wheel_radius) / max_wheel_velocity * max_duty;
    float dr = (vr / wheel_radius) / max_wheel_velocity * max_duty;

    float am = max(fabs(dl), fabs(dr));
    if (am > max_duty)
    {
        dl *= max_duty / am;
        dr *= max_duty / am;
    }
    if (am < duty_bias && am > 0.01f)
    {
        dl *= duty_bias / am;
        dr *= duty_bias / am;
    }
    if (dl > 0 && dl < duty_bias)
    {
        dl = duty_bias;
    }
    if (dl < 0 && dl > -duty_bias)
    {
        dl = -duty_bias;
    }
    if (dr > 0 && dr < duty_bias)
    {
        dr = duty_bias;
    }
    if (dr < 0 && dr > -duty_bias)
    {
        dr = -duty_bias;
    }

    left_.setDuty(dl);
    right_.setDuty(dr);
}

void Chassis::stop()
{
    left_.stop();
    right_.stop();
}

// ============================================================
// 主循环
// ============================================================
void Chassis::chassisTask()
{
    updateImu_();
    updateSensor_();
    if (checkFall_())
    {
        return;
    }

    static unsigned long tCtrl = 0;
    if (millis() - tCtrl < CONTROL_INTERVAL_MS)
    {
        return;
    }
    tCtrl = millis();

    updateMotors_();
    syncPid_();
    dispatchMode_();
    applyOutput_();
}

// ============================================================
// 子步骤
// ============================================================
void Chassis::updateImu_()
{
    static unsigned long t = 0;
    unsigned long now = millis();
    if (now - t < 5)
    {
        return;
    }
    t = now;

    float dt = (now - time_ms_) / 1000.0f;
    time_ms_ = now;
    imu_.update(dt);
    imu_state_ = imu_.getState();
    fallen_ = imu_state_.fallen;
}

void Chassis::updateSensor_()
{
    static unsigned long t = 0;
    unsigned long now = millis();
    if (now - t < SENSOR_SAMPLE_MS)
    {
        return;
    }
    t = now;
    sensor_.readRaw(raw_buf_);
    sensor_.getPos(raw_buf_, pos_, conf_);
}

bool Chassis::checkFall_()
{
    static int fr = 0;
    if (fallen_)
    {
        fr = 0;
        stop();
        state_str_ = "FALLEN";
        omega_ = 0;
        vel_ = 0;
        return true;
    }
    if (fr < 30)
    {
        fr++;
        state_str_ = "LOCKED";
        omega_ = 0;
        vel_ = 0;
        return true;
    }
    return false;
}

void Chassis::syncPid_()
{
    pid_yaw_.kp_ = yaw_kp;
    pid_yaw_.ki_ = yaw_ki;
    pid_yaw_.kd_ = yaw_kd;
    pid_yaw_.i_limit_ = yaw_i_limit;
    pid_yaw_.o_limit_ = max_angular_velocity;
}

void Chassis::updateMotors_()
{
    if (pid_enabled_)
    {
        unsigned long now = millis();
        left_.updateSpeed(now);
        right_.updateSpeed(now);
    }
}

void Chassis::applyOutput_()
{
    setVelocity(vel_, omega_);
}

// ============================================================
// 模式调度
// ============================================================
void Chassis::dispatchMode_()
{
    if (web_mode_ != prev_mode_)
    {
        pid_yaw_.reset();
        if (web_mode_ == "AUTO")
        {
            auto_state_ = AUTO_NORMAL;
        }
        else
        {
            auto_state_ = AUTO_LOCKED;
        }
        lock_count_ = 0;
        prev_mode_ = web_mode_;
    }

    if (web_mode_ == "DEBUG")
    {
        handleDebug_();
    }
    else if (web_mode_ == "STOP")
    {
        handleStop_();
    }
    else if (web_mode_ == "MANUAL")
    {
        handleManual_();
    }
    else
    {
        handleAuto_();
    }

    state_str_ = (web_mode_ == "AUTO" || web_mode_ == "DEBUG")
                     ? autoStateName_(auto_state_) : web_mode_;
}

// ============================================================
// 各模式
// ============================================================
void Chassis::handleDebug_()
{
    vel_ = debug_speed_;
    if (fabs(debug_angle_) < 1.0f)
    {
        static unsigned long st = 0;
        if (millis() - st > 1000)
        {
            yaw_ref_ += 90;
            wrapAngle_(yaw_ref_);
            st = millis();
        }
    }
    else
    {
        yaw_ref_ = debug_angle_;
    }
    omega_ = runYawPid_(yaw_ref_);
}

void Chassis::handleStop_()
{
    stop();
    vel_ = 0;
    yaw_ref_ = imu_state_.yaw;
    pid_yaw_.reset();
    auto_state_ = AUTO_LOCKED;
}

void Chassis::handleManual_()
{
    if (web_cmd_ == "LEFT")
    {
        man_ref_ += 1.57f * 0.01f * R2D;
        wrapAngle_(man_ref_);
    }
    else if (web_cmd_ == "RIGHT")
    {
        man_ref_ -= 1.57f * 0.01f * R2D;
        wrapAngle_(man_ref_);
    }
    yaw_ref_ = man_ref_;

    if (web_cmd_ == "FWD")
    {
        vel_ = target_speed;
    }
    else if (web_cmd_ == "BACK")
    {
        vel_ = -target_speed;
    }
    else
    {
        vel_ = 0;
    }

    omega_ = runYawPid_(yaw_ref_);
    auto_state_ = AUTO_LOCKED;
}

// ============================================================
// AUTO 状态机
// ============================================================
void Chassis::handleAuto_()
{
    switch (auto_state_)
    {
        case AUTO_NORMAL: autoNormal_(); break;
        case AUTO_GAP:    autoGap_();    break;
        case AUTO_LOST:   autoLost_();   break;
        case AUTO_LOCKED: autoLocked_(); break;
    }
}

void Chassis::autoNormal_()
{
    if (conf_ >= line_confidence_threshold)
    {
        last_pos_ = pos_;
        yaw_ref_ = -pos_ * 60.0f;
        vel_ = target_speed;
        omega_ = runYawPid_(yaw_ref_);
    }
    else
    {
        enterAutoState_(AUTO_GAP);
        last_omega_ = (omega_ = last_omega_ * 0.5f);
        vel_ = target_speed * 0.3f;
    }
}

void Chassis::autoGap_()
{
    if (conf_ >= line_confidence_threshold)
    {
        enterAutoState_(AUTO_NORMAL);
        yaw_ref_ = -pos_ * 60.0f;
        vel_ = target_speed;
        omega_ = runYawPid_(yaw_ref_);
        return;
    }

    bool maybe = false;
    for (int i = 1; i <= 2; i++)
    {
        if (raw_buf_[i] > 1200)
        {
            maybe = true;
        }
    }
    unsigned long grace = maybe ? (unsigned long)gap_grace_ms * 3U : (unsigned long)gap_grace_ms;

    if (millis() - gap_time_ < grace)
    {
        omega_ = last_omega_ * 0.5f;
        vel_ = target_speed * 0.3f;
    }
    else
    {
        enterAutoState_(AUTO_LOST);
        recovery_sign_ = last_pos_ < 0 ? 1 : -1;
        omega_ = 0;
    }
}

void Chassis::autoLost_()
{
    if (conf_ >= line_confidence_threshold)
    {
        enterAutoState_(AUTO_NORMAL);
        yaw_ref_ = -pos_ * 60.0f;
        vel_ = target_speed;
        omega_ = runYawPid_(yaw_ref_);
    }
    else if (millis() - lost_time_ >= (unsigned long)lost_recovery_timeout_ms)
    {
        enterAutoState_(AUTO_LOCKED);
    }
    else
    {
        float p = (float)(millis() - lost_time_) / lost_recovery_timeout_ms;
        omega_ = recovery_sign_ * (0.3f + 0.7f * p) * max_angular_velocity;
        vel_ = target_speed * 0.3f;
    }
}

void Chassis::autoLocked_()
{
    if (conf_ >= line_confidence_threshold && ++lock_count_ >= LOCKED_STABLE_COUNT)
    {
        enterAutoState_(AUTO_NORMAL);
        yaw_ref_ = -pos_ * 60.0f;
        vel_ = target_speed;
        omega_ = runYawPid_(yaw_ref_);
    }
    else
    {
        stop();
        vel_ = 0;
        omega_ = 0;
    }
}

// ============================================================
// 辅助
// ============================================================
void Chassis::enterAutoState_(AutoState s)
{
    auto_state_ = s;
    pid_yaw_.reset();
    if (s == AUTO_GAP)
    {
        gap_time_ = millis();
    }
    if (s == AUTO_LOST)
    {
        lost_time_ = millis();
    }
    if (s == AUTO_LOCKED)
    {
        lock_count_ = 0;
        stop();
    }
}

float Chassis::runYawPid_(float ref)
{
    float err = ref - imu_state_.yaw;
    wrapAngle_(err);
    float w = pid_yaw_.update(err * D2R, millis());
    if (w > max_angular_velocity)
    {
        w = max_angular_velocity;
    }
    if (w < -max_angular_velocity)
    {
        w = -max_angular_velocity;
    }
    return w;
}

void Chassis::wrapAngle_(float& v)
{
    while (v > 180)
    {
        v -= 360;
    }
    while (v < -180)
    {
        v += 360;
    }
}

String Chassis::autoStateName_(AutoState s)
{
    switch (s)
    {
        case AUTO_NORMAL: return "NORMAL";
        case AUTO_GAP:    return "GAP";
        case AUTO_LOST:   return "LOST";
        case AUTO_LOCKED: return "LOCKED";
        default:          return "?";
    }
}
