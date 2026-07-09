#include "chassis.hpp"

static const float D2R = 0.0174533f, R2D = 57.29578f;

Chassis::Chassis(Motor& l, Motor& r)
    : left_(l), right_(r),
      pid_pos_(pos_kp, pos_ki, pos_kd,
               -max_angular_velocity, max_angular_velocity, millis()),
      pid_yaw_(yaw_kp, yaw_ki, yaw_kd,
               -max_angular_velocity, max_angular_velocity, millis(), true)
{
    time_ms_ = millis();
}

void Chassis::begin()
{
    imu_.begin();
    time_ms_ = millis();
}

void Chassis::setVelocity(float v, float w)
{
    float vl = v - w * wheel_dist / 2.0f;
    float vr = v + w * wheel_dist / 2.0f;
    float dl = (vl / wheel_radius) / max_wheel_velocity * max_duty;
    float dr = (vr / wheel_radius) / max_wheel_velocity * max_duty;

    // 转弯差速偏置: 叠加到差速上 (而非保底)
    if (turn_bias > 0.01f && fabs(w) > 0.01f)
    {
        float sgn = (w > 0) ? 1.0f : -1.0f;
        dl -= sgn * turn_bias / 2.0f;
        dr += sgn * turn_bias / 2.0f;
    }

    // 差速保持式饱和: 超限时整体平移 (不按比例缩放), 保住转向权
    float hi = max(dl, dr);
    float lo = min(dl, dr);
    if (hi > max_duty)  { float s = hi - max_duty;  dl -= s; dr -= s; }
    lo = min(dl, dr);
    if (lo < -max_duty) { float s = -max_duty - lo; dl += s; dr += s; }
    if (dl > max_duty) dl = max_duty;   if (dl < -max_duty) dl = -max_duty;
    if (dr > max_duty) dr = max_duty;   if (dr < -max_duty) dr = -max_duty;

    // 死区: 保证最小启动占空比
    if (dl > 0 && dl < duty_bias) dl = duty_bias;
    if (dl < 0 && dl > -duty_bias) dl = -duty_bias;
    if (dr > 0 && dr < duty_bias) dr = duty_bias;
    if (dr < 0 && dr > -duty_bias) dr = -duty_bias;

    duty_l_ = dl;
    duty_r_ = dr;
    left_.setDuty(dl);
    right_.setDuty(dr);
}

void Chassis::stop() { left_.stop(); right_.stop(); }

void Chassis::chassisTask()
{
    updateImu_();
    updateSensor_();
    if (checkFall_()) return;

    static unsigned long tCtrl = 0;
    if (millis() - tCtrl < CONTROL_INTERVAL_MS) return;
    tCtrl = millis();

    syncPid_();
    dispatchMode_();
    setVelocity(vel_, omega_);
}

void Chassis::updateImu_()
{
    static unsigned long t = 0;
    if (millis() - t < 5) return;
    t = millis();
    float dt = (millis() - time_ms_) / 1000.0f;
    time_ms_ = millis();
    imu_.update(dt);
    imu_state_ = imu_.getState();
    fallen_ = imu_state_.fallen;
}

void Chassis::updateSensor_()
{
    static unsigned long t = 0;
    if (millis() - t < SENSOR_SAMPLE_MS) return;
    t = millis();
    sensor_.readRaw(raw_buf_);
    sensor_.getPos(raw_buf_, pos_, conf_);
}

bool Chassis::checkFall_()
{
    static int fr = 0;
    if (fallen_) { fr = 0; stop(); state_str_ = "FALLEN"; omega_ = 0; vel_ = 0; return true; }
    if (fr < 30)  { fr++;    state_str_ = "LOCKED"; omega_ = 0; vel_ = 0; return true; }
    return false;
}

void Chassis::syncPid_()
{
    // 位置 PID
    pid_pos_.kp_ = pos_kp; pid_pos_.ki_ = pos_ki; pid_pos_.kd_ = pos_kd;
    pid_pos_.i_limit_ = pos_i_limit; pid_pos_.o_limit_ = max_angular_velocity;

    // yaw 角度 PID (DEBUG)
    pid_yaw_.kp_ = yaw_kp; pid_yaw_.ki_ = yaw_ki; pid_yaw_.kd_ = yaw_kd;
    pid_yaw_.i_limit_ = yaw_i_limit; pid_yaw_.o_limit_ = max_angular_velocity;
}

// ============================================================
// 模式调度
// ============================================================
void Chassis::dispatchMode_()
{
    if (web_mode_ != prev_mode_)
    {
        pid_pos_.reset();
        pid_yaw_.reset();
        auto_state_ = (web_mode_ == "AUTO") ? AUTO_NORMAL : AUTO_LOCKED;
        lock_count_ = 0;
        prev_mode_ = web_mode_;
    }

    if      (web_mode_ == "DEBUG")  handleDebug_();
    else if (web_mode_ == "STOP")   handleStop_();
    else if (web_mode_ == "MANUAL") handleManual_();
    else                            handleAuto_();

    state_str_ = (web_mode_ == "AUTO" || web_mode_ == "DEBUG")
                     ? autoStateName_(auto_state_) : web_mode_;
}

void Chassis::handleDebug_()
{
    vel_ = debug_speed_;
    if (fabs(debug_angle_) < 1.0f)
    {
        static unsigned long st = 0;
        static float at = 0;
        if (millis() - st > 1000) { at += 90; wrapAngle_(at); st = millis(); pid_yaw_.reset(); }
        yaw_ref_ = at;
        omega_ = runYawPid_(at);
    }
    else
    {
        yaw_ref_ = debug_angle_;
        omega_ = runYawPid_(debug_angle_);
    }
}

void Chassis::handleStop_()
{
    stop();
    vel_ = 0; omega_ = 0;
    pid_pos_.reset();
    pid_yaw_.reset();
    auto_state_ = AUTO_LOCKED;
}

void Chassis::handleManual_()
{
    if (web_cmd_ == "FWD")       { vel_ = target_speed;  omega_ = 0; }
    else if (web_cmd_ == "BACK") { vel_ = -target_speed; omega_ = 0; }
    else if (web_cmd_ == "LEFT") { vel_ = 0; omega_ = max_angular_velocity; }
    else if (web_cmd_ == "RIGHT"){ vel_ = 0; omega_ = -max_angular_velocity; }
    else                         { vel_ = 0; omega_ = 0; }
    auto_state_ = AUTO_LOCKED;
}

// ============================================================
// AUTO: 位置 → 角速度 PID (跟 MicroPython 同款)
// ============================================================
void Chassis::handleAuto_()
{
    switch (auto_state_)
    {
        case AUTO_NORMAL: autoNormal_(); break;
        case AUTO_BLIND:  autoBlind_();  break;
        case AUTO_GAP:    autoGap_();    break;
        case AUTO_LOST:   autoLost_();   break;
        case AUTO_LOCKED: autoLocked_(); break;
    }
}

// 位置 PID, 三段增益调度 (同 MicroPython _compute_angular_vel)
float Chassis::runPosPid_(float pos, float dt)
{
    float error = -pos;
    float absE = fabs(pos);

    // 增益调度 + 积分衰减
    float ekp, eki, ekd;
    if (absE < 0.10f)
    {
        pos_integral_ *= 0.85f;
        ekp = pos_kp * 0.25f; eki = pos_ki * 0.1f; ekd = pos_kd * 0.3f;
    }
    else if (absE < 0.35f)
    {
        ekp = pos_kp; eki = pos_ki; ekd = pos_kd;
    }
    else
    {
        ekp = pos_kp * 1.4f; eki = pos_ki * 1.2f; ekd = pos_kd * 1.5f;
    }

    pos_integral_ += error * dt;
    if (pos_integral_ > pos_i_limit)  pos_integral_ = pos_i_limit;
    if (pos_integral_ < -pos_i_limit) pos_integral_ = -pos_i_limit;

    float deriv = (!pos_first_ && dt > 0) ? (error - pos_prev_) / dt : 0;
    pos_first_ = false; pos_prev_ = error;

    float w = ekp * error + eki * pos_integral_ + ekd * deriv;
    if (w > max_angular_velocity)  w = max_angular_velocity;
    if (w < -max_angular_velocity) w = -max_angular_velocity;
    return w;
}

float Chassis::autoSpeed_(float omega)
{
    float s = target_speed / (1.0f + turn_slow * fabs(omega) / max_angular_velocity);
    if (s < 0.05f) s = 0.05f;
    return s;
}

void Chassis::autoNormal_()
{
    if (conf_ >= line_confidence_threshold)
    {
        last_pos_ = pos_;
        omega_ = runPosPid_(pos_, 0.01f);
        last_omega_ = omega_;
        vel_ = autoSpeed_(omega_);
    }
    else if (fabs(last_pos_) < BLIND_SAFE_POS)
    {
        // 最后在内侧传感器 → 线一定还在阵列下, 只是进了盲区
        // 用 IMU 推算继续正常跑, 不限时, 不重置 PID
        auto_state_ = AUTO_BLIND;
        pred_pos_ = last_pos_;
        autoBlind_();
    }
    else
    {
        // 最后在边缘 → 线可能真冲出去了
        enterAutoState_(AUTO_GAP);
        last_omega_ = (omega_ = last_omega_ * 0.5f);
        vel_ = target_speed * 0.3f;
    }
}

void Chassis::autoBlind_()
{
    if (conf_ >= 0.6f)
    {
        auto_state_ = AUTO_NORMAL;
        last_pos_ = pos_;
        omega_ = runPosPid_(pos_, 0.01f);
        last_omega_ = omega_;
        vel_ = autoSpeed_(omega_);
        return;
    }

    float dy = (imu_state_.gyro_z) * 0.01f;
    pred_pos_ += blind_pred_gain * dy;
    if (pred_pos_ > 1.0f) pred_pos_ = 1.0f;
    if (pred_pos_ < -1.0f) pred_pos_ = -1.0f;

    if (fabs(pred_pos_) >= 1.0f)
    {
        enterAutoState_(AUTO_GAP);
        last_omega_ = (omega_ = last_omega_ * 0.5f);
        vel_ = target_speed * 0.3f;
    }
    else
    {
        // 确定没丢线, 按预测位置正常跑 (跟 NORMAL 一样: P+I 出 ω, autoSpeed 出速度)
        float error = -pred_pos_;
        float absE = fabs(pred_pos_);

        float ekp, eki;
        if (absE < 0.10f)
        {
            pos_integral_ *= 0.85f;
            ekp = pos_kp * 0.25f; eki = pos_ki * 0.1f;
        }
        else if (absE < 0.35f)
        {
            ekp = pos_kp; eki = pos_ki;
        }
        else
        {
            ekp = pos_kp * 1.4f; eki = pos_ki * 1.2f;
        }

        pos_integral_ += error * 0.01f;
        if (pos_integral_ > pos_i_limit)  pos_integral_ = pos_i_limit;
        if (pos_integral_ < -pos_i_limit) pos_integral_ = -pos_i_limit;

        float w = ekp * error + eki * pos_integral_;
        if (w > max_angular_velocity)  w = max_angular_velocity;
        if (w < -max_angular_velocity) w = -max_angular_velocity;
        omega_ = w;
        vel_ = autoSpeed_(omega_);
    }
}

void Chassis::autoGap_()
{
    if (conf_ >= line_confidence_threshold)
    {
        enterAutoState_(AUTO_NORMAL);
        last_pos_ = pos_;
        omega_ = runPosPid_(pos_, 0.01f);
        last_omega_ = omega_;
        vel_ = autoSpeed_(omega_);
        return;
    }

    // 宽限时间随"最后一次线的位置"缩放:
    //   线在中间(|last_pos|小) → 基本确定还在线上, 大幅延长宽限
    //   线在边缘(|last_pos|→1) → 可能真冲出去了, 缩短宽限
    // central=1(正中)→×4, central=0(边缘)→×1
    float central = 1.0f - fabs(last_pos_);
    if (central < 0) central = 0;
    float posScale = 1.0f + 3.0f * central;

    bool maybe = false;
    for (int i = 1; i <= 2; i++) if (raw_buf_[i] > 1200) maybe = true;
    unsigned long grace = (unsigned long)(gap_grace_ms * posScale);
    if (maybe) grace *= 3U;

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
        last_pos_ = pos_;
        omega_ = runPosPid_(pos_, 0.01f);
        last_omega_ = omega_;
        vel_ = autoSpeed_(omega_);
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
        last_pos_ = pos_;
        omega_ = runPosPid_(pos_, 0.01f);
        last_omega_ = omega_;
        vel_ = autoSpeed_(omega_);
    }
    else { stop(); vel_ = 0; omega_ = 0; }
}

// ============================================================
// 辅助
// ============================================================
void Chassis::enterAutoState_(AutoState s)
{
    auto_state_ = s;
    pos_integral_ = 0; pos_prev_ = 0; pos_first_ = true;
    if (s == AUTO_GAP) gap_time_ = millis();
    if (s == AUTO_LOST) lost_time_ = millis();
    if (s == AUTO_LOCKED) { lock_count_ = 0; stop(); }
}

float Chassis::runYawPid_(float ref)
{
    float err = ref - imu_state_.yaw;
    wrapAngle_(err);
    float w = pid_yaw_.update(err * D2R, millis());
    if (w > max_angular_velocity) w = max_angular_velocity;
    if (w < -max_angular_velocity) w = -max_angular_velocity;
    return w;
}

void Chassis::wrapAngle_(float& v) { while (v > 180) v -= 360; while (v < -180) v += 360; }

String Chassis::autoStateName_(AutoState s)
{
    switch (s)
    {
        case AUTO_NORMAL: return "NORMAL";
        case AUTO_BLIND:  return "BLIND";
        case AUTO_GAP:    return "GAP";
        case AUTO_LOST:   return "LOST";
        case AUTO_LOCKED: return "LOCKED";
        default:          return "?";
    }
}
