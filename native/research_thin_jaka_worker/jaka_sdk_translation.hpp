#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <JAKAZuRobot.h>

#include "teleop_shaping/thin_jaka_transport_adapter.hpp"

namespace research_thin_jaka {

struct ReadOnlyControllerSnapshot {
  int sdk_return_code{0};
  int controller_error_code{0};
  bool powered_on{false};
  bool enabled{false};
  bool estop{false};
  bool collision{false};
  bool servo_move{false};
  int tool_id{-1};
  int user_frame_id{-1};
  int collision_level{-1};
  double payload_mass_kg{0.0};
  std::array<double, 3> payload_com_mm{};
  std::array<double, 3> installation_rpy_rad{};
  std::array<double, 6> active_tcp_mm_rpy_rad{};
};

class JakaSdkTranslation final {
 public:
  JakaSdkTranslation(std::string robot_ip, std::string edg_state_ip) noexcept;
  JakaSdkTranslation(const JakaSdkTranslation&) = delete;
  JakaSdkTranslation& operator=(const JakaSdkTranslation&) = delete;

  teleop_shaping::JakaSdkFunctionTable FunctionTable() noexcept;
  bool ReadAndVerifyPreflight(int expected_tool_id, int expected_user_frame_id,
                              double expected_payload_mass_kg,
                              const std::array<double, 3>& expected_com_mm,
                              double payload_tolerance_kg,
                              double com_tolerance_mm,
                              ReadOnlyControllerSnapshot* snapshot) noexcept;
  int last_sdk_return_code() const noexcept { return last_sdk_return_code_; }
  const char* last_operation() const noexcept { return last_operation_; }

 private:
  static teleop_shaping::JakaFunctionResult Login(void* context) noexcept;
  static teleop_shaping::JakaFunctionResult SetEdg(void* context,
                                                   bool enabled) noexcept;
  static teleop_shaping::JakaFunctionResult SetServo(void* context,
                                                     bool enabled) noexcept;
  static teleop_shaping::JakaFunctionResult Send(void* context,
      const double* joints, std::uint8_t dof, std::uint32_t step_num) noexcept;
  static teleop_shaping::JakaFunctionResult ReadFeedback(
      void* context, teleop_shaping::JakaJointFeedback* feedback) noexcept;
  static teleop_shaping::JakaFunctionResult ReadStatus(
      void* context, teleop_shaping::JakaNormalizedStatus* status) noexcept;
  static teleop_shaping::JakaFunctionResult StopMotion(void* context) noexcept;
  static teleop_shaping::JakaFunctionResult Logout(void* context) noexcept;

  teleop_shaping::JakaFunctionResult Record(errno_t code,
                                             const char* operation) noexcept;

  JAKAZuRobot robot_;
  std::string robot_ip_;
  std::string edg_state_ip_;
  bool logged_in_{false};
  bool edg_enabled_{false};
  bool servo_enabled_{false};
  std::uint64_t feedback_sequence_{0};
  std::uint64_t status_sequence_{0};
  int last_sdk_return_code_{0};
  const char* last_operation_{"none"};
};

}  // namespace research_thin_jaka
