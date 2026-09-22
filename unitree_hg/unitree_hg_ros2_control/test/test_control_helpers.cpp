// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "unitree_hg_ros2_control/control_helpers.hpp"
#include "unitree_hg_ros2_control/diagnostics_context.hpp"
#include "unitree_hg_ros2_control/joint_index_tables.hpp"
#include "unitree/idl/hg/HandCmd_.hpp"

namespace unitree_hg_ros2_control
{
namespace detail
{
struct BlockingSnapshot
{
  BlockingSnapshot() = default;
  BlockingSnapshot(const BlockingSnapshot &) = default;
  BlockingSnapshot(BlockingSnapshot &&) = default;

  BlockingSnapshot & operator=(const BlockingSnapshot & other)
  {
    assign(other);
    return *this;
  }

  BlockingSnapshot & operator=(BlockingSnapshot && other)
  {
    assign(other);
    return *this;
  }

  void assign(const BlockingSnapshot & other)
  {
    if (other.assignment_entered != nullptr && other.release_assignment != nullptr) {
      other.assignment_entered->store(true);
      while (!other.release_assignment->load()) {
        std::this_thread::yield();
      }
    }
    generation = other.generation;
  }

  int generation{0};
  std::atomic<bool> * assignment_entered{nullptr};
  std::atomic<bool> * release_assignment{nullptr};
};

inline bool copy_snapshot_without_allocation(
  const BlockingSnapshot & source, BlockingSnapshot & destination)
{
  destination.generation = source.generation;
  return true;
}
}  // namespace detail

namespace
{
TEST(StateHandoff, RealtimeReadDoesNotWaitForContendedWriter)
{
  detail::StateHandoff<detail::BlockingSnapshot> handoff;
  detail::BlockingSnapshot initial;
  initial.generation = 1;
  handoff.publish(initial);

  detail::BlockingSnapshot primed;
  ASSERT_TRUE(handoff.copy_latest_without_allocation(primed));
  ASSERT_EQ(primed.generation, 1);

  std::atomic<bool> assignment_entered{false};
  std::atomic<bool> release_assignment{false};
  detail::BlockingSnapshot update;
  update.generation = 2;
  update.assignment_entered = &assignment_entered;
  update.release_assignment = &release_assignment;
  std::thread writer([&]() {handoff.publish(update);});

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!assignment_entered.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_TRUE(assignment_entered.load());

  auto reader = std::async(std::launch::async, [&]() {
        detail::BlockingSnapshot snapshot;
        const bool copied = handoff.copy_latest_without_allocation(snapshot);
        return std::make_pair(copied, snapshot.generation);
    });
  EXPECT_EQ(reader.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);

  release_assignment.store(true);
  writer.join();
  const auto [copied, generation] = reader.get();
  EXPECT_TRUE(copied);
  EXPECT_EQ(generation, 1);
}

TEST(DiagnosticsContext, CapturedContextSurvivesOwnerTeardown)
{
  std::vector<detail::DiagnosticJointMetadata> joints{{"hip_joint", 0}};
  auto owner = std::make_shared<detail::DiagnosticsContext>(
    std::move(joints), 80, rclcpp::get_logger("diagnostics_context_test"));
  owner->temperatures[0].surface.store(42);
  const std::weak_ptr<detail::DiagnosticsContext> lifetime = owner;

  std::promise<void> callback_started;
  std::promise<void> release_callback;
  auto release_future = release_callback.get_future().share();
  auto callback = std::async(
    std::launch::async,
    [context = owner, &callback_started, release_future]() {
      callback_started.set_value();
      release_future.wait();
      return context->temperatures[0].surface.load();
    });

  callback_started.get_future().wait();
  owner.reset();
  EXPECT_FALSE(lifetime.expired());
  release_callback.set_value();
  EXPECT_EQ(callback.get(), 42);
}

TEST(StateHandoff, CopiesOnlyCoherentSnapshotsWithoutReallocation)
{
  detail::StateHandoff<detail::BodyStateSnapshot> handoff;
  detail::BodyStateSnapshot initial;
  initial.joints.resize(8);
  handoff.publish(initial);

  detail::BodyStateSnapshot snapshot;
  snapshot.joints.resize(8);
  const auto * const storage = snapshot.joints.data();
  std::thread writer([&]() {
      for (int generation = 1; generation <= 10000; ++generation) {
        detail::BodyStateSnapshot update;
        update.joints.resize(8);
        for (auto & joint : update.joints) {
          joint.position = generation;
        }
        handoff.publish(std::move(update));
      }
    });
  for (int read = 0; read < 10000; ++read) {
    ASSERT_TRUE(handoff.copy_latest_without_allocation(snapshot));
    EXPECT_EQ(snapshot.joints.data(), storage);
    for (const auto & joint : snapshot.joints) {
      EXPECT_EQ(joint.position, snapshot.joints.front().position);
    }
  }
  writer.join();
}

TEST(StateHandoff, Dex3CopiesIntoStablePresizedStorage)
{
  detail::StateHandoff<std::vector<detail::JointStateSample>> handoff;
  handoff.publish(std::vector<detail::JointStateSample>(7));
  std::vector<detail::JointStateSample> snapshot(7);
  const auto * const storage = snapshot.data();

  for (int generation = 1; generation <= 1000; ++generation) {
    std::vector<detail::JointStateSample> update(7);
    for (auto & joint : update) {
      joint.position = generation;
    }
    handoff.publish(std::move(update));
    ASSERT_TRUE(handoff.copy_latest_without_allocation(snapshot));
    EXPECT_EQ(snapshot.data(), storage);
    EXPECT_EQ(snapshot.front().position, generation);
    EXPECT_EQ(snapshot.back().position, generation);
  }
}

TEST(HandStateValidation, RequiresEveryConfiguredSdkIndex)
{
  std::vector<JointData> joints(7);
  for (int index = 0; index < 7; ++index) {
    joints[index].sdk_index = index;
  }
  EXPECT_FALSE(detail::hand_state_covers_configured_joints(0, joints));
  EXPECT_FALSE(detail::hand_state_covers_configured_joints(6, joints));
  EXPECT_TRUE(detail::hand_state_covers_configured_joints(7, joints));
}

TEST(ChannelRollback, CleansChildrenBeforeReleaseAndLeaseCanBeReacquired)
{
  std::vector<std::string> events;
  bool lease_held = false;
  bool child_live = false;
  auto acquire = [&]() {
      if (lease_held) {
        return false;
      }
      lease_held = true;
      events.emplace_back("acquire");
      return true;
    };

  ASSERT_TRUE(acquire());
  child_live = true;
  detail::rollback_channel_initialization(
    [&]() {
      EXPECT_TRUE(lease_held);
      child_live = false;
      events.emplace_back("children");
    },
    [&]() {
      EXPECT_FALSE(child_live);
      lease_held = false;
      events.emplace_back("release");
    });
  EXPECT_EQ(events, (std::vector<std::string>{"acquire", "children", "release"}));

  ASSERT_TRUE(acquire());
  child_live = true;
  EXPECT_TRUE(lease_held);
  EXPECT_TRUE(child_live);
  EXPECT_EQ(events.back(), "acquire");
}

TEST(CommandClaims, KpAndKdAreTrackedIndependently)
{
  JointData joint;
  detail::set_command_interface_claim(joint, HW_IF_KP, true);
  EXPECT_FALSE(joint.is_impedance_control_enabled);
  detail::set_command_interface_claim(joint, HW_IF_KD, true);
  EXPECT_TRUE(joint.is_impedance_control_enabled);
  detail::set_command_interface_claim(joint, HW_IF_KP, false);
  EXPECT_FALSE(joint.is_impedance_control_enabled);
  EXPECT_TRUE(joint.is_kd_claimed);
  detail::set_command_interface_claim(joint, HW_IF_KP, true);
  detail::set_command_interface_claim(joint, HW_IF_KD, false);
  EXPECT_FALSE(joint.is_impedance_control_enabled);
  EXPECT_TRUE(joint.is_kp_claimed);
}

TEST(Dex3Mode, PacksActiveAndInactiveBytesForAllMotors)
{
  for (uint8_t motor = 0; motor < 7; ++motor) {
    EXPECT_EQ(detail::pack_dex3_mode(motor, 1, false), 0x10 | motor);
    EXPECT_EQ(detail::pack_dex3_mode(motor, 1, true), 0x90 | motor);
    EXPECT_EQ(detail::pack_dex3_mode(motor | 0xF0, 1, false), 0x10 | motor);
  }
}

TEST(CommandComposition, Dex3UsesHandGainsAndImpedanceValues)
{
  JointData joint;
  joint.position_state = 0.25;
  joint.position_command = 0.5;
  joint.is_position_control_enabled = true;
  detail::MotorCommand command;
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 2, command));
  EXPECT_EQ(command.mode, 0x12);
  EXPECT_FLOAT_EQ(command.kp, 0.5f);
  EXPECT_FLOAT_EQ(command.kd, 0.1f);
  joint.is_impedance_control_enabled = true;
  joint.kp_command = 3.0;
  joint.kd_command = 0.4;
  joint.velocity_command = 0.6;
  joint.effort_command = 0.7;
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 2, command));
  EXPECT_FLOAT_EQ(command.kp, 3.0f);
  EXPECT_FLOAT_EQ(command.kd, 0.4f);
}

TEST(CommandComposition, Dex3BackingStorageIsStableAcrossCycles)
{
  unitree_hg::msg::dds_::HandCmd_ hand_command;
  hand_command.motor_cmd().resize(7);
  const auto * const storage = hand_command.motor_cmd().data();
  JointData joint;
  joint.is_position_control_enabled = true;
  for (int cycle = 0; cycle < 100; ++cycle) {
    detail::MotorCommand command;
    ASSERT_TRUE(detail::compose_motor_command(
      joint, detail::CommandFlavor::kDex3, cycle % 7, command));
    hand_command.motor_cmd()[cycle % 7].q() = command.q;
    EXPECT_EQ(hand_command.motor_cmd().data(), storage);
  }
}

TEST(CommandComposition, Dex3OmittedSlotsAreInactiveAndZeroEveryCycle)
{
  unitree_hg::msg::dds_::HandCmd_ hand_command;
  hand_command.motor_cmd().resize(7);
  const auto * const storage = hand_command.motor_cmd().data();

  for (auto & motor : hand_command.motor_cmd()) {
    motor.mode() = 0x11;
    motor.q() = 1.0f;
    motor.dq() = 2.0f;
    motor.tau() = 3.0f;
    motor.kp() = 4.0f;
    motor.kd() = 5.0f;
    motor.reserve() = 6;
  }

  detail::initialize_dex3_command_slots(hand_command.motor_cmd());
  JointData configured;
  configured.is_position_control_enabled = true;
  configured.position_command = 0.25;
  detail::MotorCommand configured_command;
  ASSERT_TRUE(detail::compose_motor_command(
    configured, detail::CommandFlavor::kDex3, 2, configured_command));
  auto & configured_motor = hand_command.motor_cmd()[2];
  configured_motor.mode() = configured_command.mode;
  configured_motor.q() = configured_command.q;
  configured_motor.kp() = configured_command.kp;
  configured_motor.kd() = configured_command.kd;

  EXPECT_EQ(hand_command.motor_cmd().data(), storage);
  EXPECT_EQ(configured_motor.mode(), 0x12);
  for (uint8_t motor_id = 0; motor_id < 7; ++motor_id) {
    if (motor_id == 2) {
      continue;
    }
    const auto & omitted = hand_command.motor_cmd()[motor_id];
    EXPECT_EQ(omitted.mode(), 0x90 | motor_id);
    EXPECT_FLOAT_EQ(omitted.q(), 0.0f);
    EXPECT_FLOAT_EQ(omitted.dq(), 0.0f);
    EXPECT_FLOAT_EQ(omitted.tau(), 0.0f);
    EXPECT_FLOAT_EQ(omitted.kp(), 0.0f);
    EXPECT_FLOAT_EQ(omitted.kd(), 0.0f);
    EXPECT_EQ(omitted.reserve(), 0U);
  }

  detail::initialize_dex3_command_slots(hand_command.motor_cmd());
  EXPECT_EQ(hand_command.motor_cmd().data(), storage);
  EXPECT_EQ(hand_command.motor_cmd()[2].mode(), 0x92);
  EXPECT_FLOAT_EQ(hand_command.motor_cmd()[2].q(), 0.0f);
  EXPECT_FLOAT_EQ(hand_command.motor_cmd()[2].kp(), 0.0f);
}

TEST(CommandComposition, VelocityOnlyPropagatesAndDisabledIsSafe)
{
  JointData joint;
  joint.position_state = 1.25;
  joint.velocity_command = -0.75;
  joint.is_velocity_control_enabled = true;
  detail::MotorCommand body;
  detail::MotorCommand hand;
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 3, body));
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 3, hand));
  EXPECT_EQ(body.mode, 1);
  EXPECT_EQ(hand.mode, 0x13);
  EXPECT_FLOAT_EQ(body.q, 1.25f);
  EXPECT_FLOAT_EQ(body.dq, -0.75f);
  EXPECT_FLOAT_EQ(body.kd, 1.0f);
  joint.is_velocity_control_enabled = false;
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 3, hand));
  EXPECT_EQ(hand.mode, 0x93);
  EXPECT_FLOAT_EQ(hand.q, 0.0f);
  EXPECT_FLOAT_EQ(hand.dq, 0.0f);
  EXPECT_FLOAT_EQ(hand.tau, 0.0f);
  EXPECT_FLOAT_EQ(hand.kp, 0.0f);
  EXPECT_FLOAT_EQ(hand.kd, 0.0f);
}

TEST(CommandComposition, RejectsNonFiniteOutboundFieldsAndFloatOverflow)
{
  JointData joint;
  joint.is_impedance_control_enabled = true;
  joint.position_command = 1.0;
  joint.velocity_command = 2.0;
  joint.effort_command = 3.0;
  joint.kp_command = 4.0;
  joint.kd_command = 5.0;
  detail::MotorCommand command;
  ASSERT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));
  double * fields[] = {&joint.position_command, &joint.velocity_command,
    &joint.effort_command, &joint.kp_command, &joint.kd_command};
  for (auto * field : fields) {
    const double saved = *field;
    *field = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));
    *field = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));
    *field = saved;
  }
  joint.position_command = std::numeric_limits<double>::max();
  EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));
}

TEST(CommandComposition, ValidatesActualFieldsInEveryEnabledMode)
{
  JointData joint;
  detail::MotorCommand command;

  joint.is_position_control_enabled = true;
  joint.position_command = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));

  joint = JointData{};
  joint.is_velocity_control_enabled = true;
  joint.position_state = 1.0;
  joint.velocity_command = 2.0;
  EXPECT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));
  joint.position_state = std::numeric_limits<double>::max();
  EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kBody, 0, command));

  joint = JointData{};
  joint.is_effort_control_enabled = true;
  joint.position_state = 1.0;
  joint.velocity_command = 2.0;
  joint.effort_command = 3.0;
  joint.kd_command = 4.0;
  EXPECT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 0, command));
  joint.effort_command = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 0, command));

  joint = JointData{};
  joint.position_command = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(detail::compose_motor_command(joint, detail::CommandFlavor::kDex3, 0, command));
  EXPECT_EQ(command.mode, 0x90);
}

TEST(ConfigurationValidation, Dex3JointSetMustExactlyMatchSelectedHand)
{
  const std::vector<std::string> left{
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
  };
  const std::vector<std::string> right{
    "right_hand_index_1_joint",
    "right_hand_index_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_thumb_2_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_0_joint",
  };

  EXPECT_TRUE(detail::hand_joint_names_match_side("left", left));
  EXPECT_TRUE(detail::hand_joint_names_match_side("right", right));
  EXPECT_FALSE(detail::hand_joint_names_match_side("", left));
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", {}));

  auto partial = left;
  partial.pop_back();
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", partial));

  auto mixed_side = left;
  mixed_side.back() = "right_hand_index_1_joint";
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", mixed_side));

  auto duplicate = left;
  duplicate.back() = duplicate.front();
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", duplicate));

  auto unknown = left;
  unknown.back() = "left_hand_pinky_0_joint";
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", unknown));

  auto extra = left;
  extra.push_back("left_hand_pinky_0_joint");
  EXPECT_FALSE(detail::hand_joint_names_match_side("left", extra));
}

std::vector<std::string> names_from_table(const JointIndexTable & table)
{
  std::vector<std::string> names;
  for (const auto & entry : table) {
    names.push_back(entry.first);
  }
  return names;
}

TEST(ConfigurationValidation, BodyJointSetMustExactlyMatchVariant)
{
  const auto g1 = names_from_table(g1_body_joint_index_table());
  const auto h2 = names_from_table(h2_body_joint_index_table());
  EXPECT_TRUE(detail::joint_names_match_table(g1, g1_body_joint_index_table()));
  EXPECT_TRUE(detail::joint_names_match_table(h2, h2_body_joint_index_table()));
  EXPECT_FALSE(detail::joint_names_match_table(g1, h2_body_joint_index_table()));
}
}  // namespace
}  // namespace unitree_hg_ros2_control
