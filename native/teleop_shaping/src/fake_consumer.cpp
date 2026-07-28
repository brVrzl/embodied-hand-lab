#include "teleop_shaping/fake_consumer.hpp"

#include <cassert>

namespace teleop_shaping {

InMemoryFakeConsumerV1::InMemoryFakeConsumerV1(std::uint8_t dof,
                                               std::uint64_t safety_epoch) noexcept
    : dof_(dof),
      safety_epoch_(safety_epoch),
      highest_offered_sequence_(0),
      last_consumed_sequence_(0),
      superseded_count_(0),
      has_pending_(false),
      hard_stopped_(dof == 0U || dof > teleop_command_abi::kMaxDof || safety_epoch == 0U),
      pending_{},
      telemetry_{},
      telemetry_size_(0),
      telemetry_next_(0) {}

void InMemoryFakeConsumerV1::Record(std::uint64_t offered, ConsumerCode code,
                                    teleop_command_abi::OutputMode mode,
                                    std::int64_t now_ns) noexcept {
  telemetry_[telemetry_next_] =
      ConsumerTelemetry{offered, last_consumed_sequence_, now_ns, code, mode};
  telemetry_next_ = (telemetry_next_ + 1U) % kTelemetryCapacity;
  if (telemetry_size_ < kTelemetryCapacity) ++telemetry_size_;
}

ConsumerCode InMemoryFakeConsumerV1::Offer(
    const teleop_command_abi::ShapedJointCommandV1& command,
    std::int64_t now_ns) noexcept {
  if (hard_stopped_) {
    Record(command.output_sequence, ConsumerCode::kHardStopped, command.output_mode, now_ns);
    return ConsumerCode::kHardStopped;
  }
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_dof = dof_;
  context.expected_epoch = safety_epoch_;
  context.previous_sequence = highest_offered_sequence_;
  const teleop_command_abi::ValidationResult validation =
      teleop_command_abi::Validate(command, context);
  if (!validation.ok) {
    ConsumerCode code = ConsumerCode::kInvalidCommand;
    if (validation.error == teleop_command_abi::ValidationError::kEpochMismatch) {
      code = ConsumerCode::kEpochMismatch;
    } else if (validation.error ==
               teleop_command_abi::ValidationError::kStaleValidityWindow) {
      code = ConsumerCode::kStaleCommand;
    } else if (validation.error ==
               teleop_command_abi::ValidationError::kInvalidSequence) {
      code = ConsumerCode::kDuplicateOrOld;
    }
    hard_stopped_ = code != ConsumerCode::kDuplicateOrOld;
    has_pending_ = false;
    Record(command.output_sequence, code, command.output_mode, now_ns);
    return code;
  }
  if (command.output_mode == teleop_command_abi::OutputMode::kHardStopped ||
      command.output_mode == teleop_command_abi::OutputMode::kInactive) {
    hard_stopped_ = true;
    has_pending_ = false;
    Record(command.output_sequence, ConsumerCode::kHardStopped, command.output_mode, now_ns);
    return ConsumerCode::kHardStopped;
  }
  if (has_pending_) ++superseded_count_;
  pending_ = command;
  has_pending_ = true;
  highest_offered_sequence_ = command.output_sequence;
  Record(command.output_sequence, ConsumerCode::kOk, command.output_mode, now_ns);
  return ConsumerCode::kOk;
}

ConsumerCode InMemoryFakeConsumerV1::ConsumeLatest(std::int64_t now_ns) noexcept {
  if (hard_stopped_) {
    Record(0U, ConsumerCode::kHardStopped, teleop_command_abi::OutputMode::kHardStopped,
           now_ns);
    return ConsumerCode::kHardStopped;
  }
  if (!has_pending_) {
    Record(0U, ConsumerCode::kNoCommand, teleop_command_abi::OutputMode::kInactive,
           now_ns);
    return ConsumerCode::kNoCommand;
  }
  if (pending_.valid_until_monotonic_ns < now_ns) {
    hard_stopped_ = true;
    has_pending_ = false;
    Record(pending_.output_sequence, ConsumerCode::kStaleCommand, pending_.output_mode,
           now_ns);
    return ConsumerCode::kStaleCommand;
  }
  last_consumed_sequence_ = pending_.output_sequence;
  const auto mode = pending_.output_mode;
  has_pending_ = false;
  Record(last_consumed_sequence_, ConsumerCode::kOk, mode, now_ns);
  return ConsumerCode::kOk;
}

void InMemoryFakeConsumerV1::InjectProducerDisappearance() noexcept {
  hard_stopped_ = true;
  has_pending_ = false;
}

void InMemoryFakeConsumerV1::HardStop() noexcept {
  hard_stopped_ = true;
  has_pending_ = false;
}

const ConsumerTelemetry& InMemoryFakeConsumerV1::telemetry(std::size_t index) const noexcept {
  assert(index < telemetry_size_);
  const std::size_t oldest =
      telemetry_size_ < kTelemetryCapacity ? 0U : telemetry_next_;
  return telemetry_[(oldest + index) % kTelemetryCapacity];
}

}  // namespace teleop_shaping
