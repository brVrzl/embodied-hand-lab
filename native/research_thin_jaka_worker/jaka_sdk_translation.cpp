#include "jaka_sdk_translation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <jkerr.h>

namespace research_thin_jaka {
namespace {

std::int64_t NowNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool Near(double a, double b, double tolerance) noexcept {
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= tolerance;
}

}  // namespace

JakaSdkTranslation::JakaSdkTranslation(std::string robot_ip,
                                       std::string edg_state_ip) noexcept
    : robot_ip_(std::move(robot_ip)), edg_state_ip_(std::move(edg_state_ip)) {}

teleop_shaping::JakaSdkFunctionTable JakaSdkTranslation::FunctionTable() noexcept {
  return {this, &Login, &SetEdg, &SetServo, &Send, &ReadFeedback,
          &ReadStatus, &StopMotion, &Logout};
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::Record(
    errno_t code, const char* operation) noexcept {
  last_sdk_return_code_ = code;
  last_operation_ = operation;
  return code == ERR_SUCC ? teleop_shaping::JakaFunctionResult::kOk
                          : teleop_shaping::JakaFunctionResult::kFailure;
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::Login(void* context) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  const errno_t code = self.robot_.login_in(self.robot_ip_.c_str());
  if (code == ERR_SUCC) self.logged_in_ = true;
  return self.Record(code, "login_in");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::SetEdg(
    void* context, bool enabled) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (self.edg_enabled_ == enabled) return teleop_shaping::JakaFunctionResult::kOk;
  const errno_t code = self.robot_.edg_init(enabled ? TRUE : FALSE,
                                             self.edg_state_ip_.c_str());
  if (code == ERR_SUCC) self.edg_enabled_ = enabled;
  return self.Record(code, enabled ? "edg_init(true)" : "edg_init(false)");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::SetServo(
    void* context, bool enabled) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (self.servo_enabled_ == enabled) return teleop_shaping::JakaFunctionResult::kOk;
  const errno_t code = self.robot_.servo_move_enable(enabled ? TRUE : FALSE);
  if (code == ERR_SUCC) self.servo_enabled_ = enabled;
  return self.Record(code, enabled ? "servo_move_enable(true)"
                                   : "servo_move_enable(false)");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::Send(
    void* context, const double* joints, std::uint8_t dof,
    std::uint32_t step_num) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (joints == nullptr || dof != 6U || step_num == 0U ||
      !self.logged_in_ || !self.edg_enabled_ || !self.servo_enabled_) {
    return teleop_shaping::JakaFunctionResult::kRejected;
  }
  JointValue command{};
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(joints[i])) return teleop_shaping::JakaFunctionResult::kRejected;
    command.jVal[i] = joints[i];
  }
  return self.Record(self.robot_.edg_servo_j(&command, ABS, step_num),
                     "edg_servo_j");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::ReadFeedback(
    void* context, teleop_shaping::JakaJointFeedback* feedback) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (feedback == nullptr || !self.logged_in_ || !self.edg_enabled_) {
    return teleop_shaping::JakaFunctionResult::kRejected;
  }
  EDGState state{};
  const std::int64_t call_start_ns = NowNs();
  const errno_t code = self.robot_.edg_get_stat(&state);
  const std::int64_t call_end_ns = NowNs();
  if (code != ERR_SUCC) return self.Record(code, "edg_get_stat");
  teleop_shaping::JakaJointFeedback result{};
  result.sequence = ++self.feedback_sequence_;
  result.sdk_call_start_monotonic_ns = call_start_ns;
  result.sdk_call_end_monotonic_ns = call_end_ns;
  // The installed API supplies no controller acquisition timestamp. The
  // narrowest honest host bound is SDK-call completion, recorded separately
  // from the subsequent validation timestamp.
  result.sampled_monotonic_ns = call_end_ns;
  result.dof = 6;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(state.jointVal.jVal[i]) ||
        !std::isfinite(state.jointVel.jVel[i])) {
      return teleop_shaping::JakaFunctionResult::kFailure;
    }
    result.position_rad[i] = state.jointVal.jVal[i];
    result.velocity_rad_s[i] = state.jointVel.jVel[i];
  }
  result.validation_monotonic_ns = NowNs();
  *feedback = result;
  return self.Record(code, "edg_get_stat");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::ReadStatus(
    void* context, teleop_shaping::JakaNormalizedStatus* status) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (status == nullptr || !self.logged_in_) {
    return teleop_shaping::JakaFunctionResult::kRejected;
  }
  RobotStatus_simple raw{};
  const std::int64_t call_start_ns = NowNs();
  const errno_t code = self.robot_.get_robot_status_simple(&raw);
  const std::int64_t call_end_ns = NowNs();
  if (code != ERR_SUCC) return self.Record(code, "get_robot_status_simple");
  teleop_shaping::JakaNormalizedStatus result{};
  result.sequence = ++self.status_sequence_;
  result.sdk_call_start_monotonic_ns = call_start_ns;
  result.sdk_call_end_monotonic_ns = call_end_ns;
  result.sampled_monotonic_ns = call_end_ns;
  result.session_alive = true;
  result.powered_on = raw.powered_on != 0;
  result.servo_enabled = raw.enabled != 0 && self.servo_enabled_;
  result.edg_ready = self.edg_enabled_;
  result.alarm = raw.errcode != 0;
  if (result.alarm || !result.powered_on || !result.servo_enabled) {
    BOOL estop = FALSE;
    BOOL collision = FALSE;
    const errno_t estop_code = self.robot_.is_in_estop(&estop);
    const errno_t collision_code = self.robot_.is_in_collision(&collision);
    if (estop_code != ERR_SUCC || collision_code != ERR_SUCC) {
      return self.Record(estop_code != ERR_SUCC ? estop_code : collision_code,
                         estop_code != ERR_SUCC ? "is_in_estop"
                                                : "is_in_collision");
    }
    result.estop = estop != FALSE;
    result.collision = collision != FALSE;
  }
  result.validation_monotonic_ns = NowNs();
  *status = result;
  return self.Record(code, "get_robot_status_simple");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::StopMotion(
    void* context) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (!self.servo_enabled_) return teleop_shaping::JakaFunctionResult::kOk;
  const errno_t code = self.robot_.servo_move_enable(FALSE);
  if (code == ERR_SUCC) self.servo_enabled_ = false;
  return self.Record(code, "servo_move_enable(false):hard_stop");
}

teleop_shaping::JakaFunctionResult JakaSdkTranslation::Logout(void* context) noexcept {
  auto& self = *static_cast<JakaSdkTranslation*>(context);
  if (!self.logged_in_) return teleop_shaping::JakaFunctionResult::kOk;
  const errno_t code = self.robot_.login_out();
  if (code == ERR_SUCC) self.logged_in_ = false;
  return self.Record(code, "login_out");
}

bool JakaSdkTranslation::ReadAndVerifyPreflight(
    int expected_tool_id, int expected_user_frame_id,
    double expected_payload_mass_kg, const std::array<double, 3>& expected_com_mm,
    double payload_tolerance_kg, double com_tolerance_mm,
    ReadOnlyControllerSnapshot* snapshot) noexcept {
  if (!logged_in_ || snapshot == nullptr) return false;
  ReadOnlyControllerSnapshot value{};
  RobotStatus_simple status{};
  PayLoad payload{};
  Quaternion installation_quaternion{};
  Rpy installation_rpy{};
  CartesianPose tcp{};
  BOOL estop = FALSE;
  BOOL collision = FALSE;
  BOOL servo = FALSE;
  auto check = [&](errno_t code, const char* operation) {
    value.sdk_return_code = code;
    if (code != ERR_SUCC) {
      Record(code, operation);
      return false;
    }
    return true;
  };
  if (!check(robot_.get_robot_status_simple(&status), "get_robot_status_simple") ||
      !check(robot_.is_in_estop(&estop), "is_in_estop") ||
      !check(robot_.is_in_collision(&collision), "is_in_collision") ||
      !check(robot_.is_in_servomove(&servo), "is_in_servomove") ||
      !check(robot_.get_tool_id(&value.tool_id), "get_tool_id") ||
      !check(robot_.get_user_frame_id(&value.user_frame_id), "get_user_frame_id") ||
      !check(robot_.get_collision_level(&value.collision_level), "get_collision_level") ||
      !check(robot_.get_payload(&payload), "get_payload") ||
      !check(robot_.get_installation_angle(&installation_quaternion,
                                            &installation_rpy),
             "get_installation_angle") ||
      !check(robot_.get_tool_data(value.tool_id, &tcp), "get_tool_data")) {
    *snapshot = value;
    return false;
  }
  value.controller_error_code = status.errcode;
  value.powered_on = status.powered_on != 0;
  value.enabled = status.enabled != 0;
  value.estop = estop != FALSE;
  value.collision = collision != FALSE;
  value.servo_move = servo != FALSE;
  value.payload_mass_kg = payload.mass;
  value.payload_com_mm = {payload.centroid.x, payload.centroid.y, payload.centroid.z};
  value.installation_rpy_rad = {installation_rpy.rx, installation_rpy.ry,
                                installation_rpy.rz};
  value.active_tcp_mm_rpy_rad = {tcp.tran.x, tcp.tran.y, tcp.tran.z,
                                 tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz};
  *snapshot = value;
  bool tcp_zero = true;
  for (double component : value.active_tcp_mm_rpy_rad) {
    tcp_zero = tcp_zero && Near(component, 0.0, 1e-6);
  }
  bool installation_zero = true;
  for (double component : value.installation_rpy_rad) {
    installation_zero = installation_zero && Near(component, 0.0, 1e-6);
  }
  bool com_ok = true;
  for (std::size_t i = 0; i < 3; ++i) {
    com_ok = com_ok && Near(value.payload_com_mm[i], expected_com_mm[i],
                            com_tolerance_mm);
  }
  return status.errcode == 0 && value.powered_on && value.enabled &&
         !value.estop && !value.collision && !value.servo_move &&
         value.tool_id == expected_tool_id &&
         value.user_frame_id == expected_user_frame_id &&
         Near(value.payload_mass_kg, expected_payload_mass_kg,
              payload_tolerance_kg) &&
         com_ok && tcp_zero && installation_zero && value.collision_level >= 1;
}

}  // namespace research_thin_jaka
