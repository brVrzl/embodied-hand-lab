#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "teleop_command_abi/abi_v1.hpp"

namespace teleop_shaping {

enum class ConsumerCode : std::uint8_t {
  kOk = 0,
  kNoCommand,
  kDuplicateOrOld,
  kInvalidCommand,
  kStaleCommand,
  kEpochMismatch,
  kHardStopped,
};

struct ConsumerTelemetry {
  std::uint64_t offered_sequence;
  std::uint64_t consumed_sequence;
  std::int64_t sampled_monotonic_ns;
  ConsumerCode code;
  teleop_command_abi::OutputMode mode;
};

class InMemoryFakeConsumerV1 {
 public:
  static constexpr std::size_t kTelemetryCapacity = 256;

  InMemoryFakeConsumerV1(std::uint8_t dof, std::uint64_t safety_epoch) noexcept;
  ConsumerCode Offer(const teleop_command_abi::ShapedJointCommandV1& command,
                     std::int64_t now_ns) noexcept;
  ConsumerCode ConsumeLatest(std::int64_t now_ns) noexcept;
  void InjectProducerDisappearance() noexcept;
  void HardStop() noexcept;
  bool hard_stopped() const noexcept { return hard_stopped_; }
  std::uint64_t last_consumed_sequence() const noexcept { return last_consumed_sequence_; }
  std::uint64_t superseded_count() const noexcept { return superseded_count_; }
  std::size_t telemetry_size() const noexcept { return telemetry_size_; }
  const ConsumerTelemetry& telemetry(std::size_t index) const noexcept;

 private:
  void Record(std::uint64_t offered, ConsumerCode code,
              teleop_command_abi::OutputMode mode, std::int64_t now_ns) noexcept;

  std::uint8_t dof_;
  std::uint64_t safety_epoch_;
  std::uint64_t highest_offered_sequence_;
  std::uint64_t last_consumed_sequence_;
  std::uint64_t superseded_count_;
  bool has_pending_;
  bool hard_stopped_;
  teleop_command_abi::ShapedJointCommandV1 pending_;
  std::array<ConsumerTelemetry, kTelemetryCapacity> telemetry_;
  std::size_t telemetry_size_;
  std::size_t telemetry_next_;
};

}  // namespace teleop_shaping
