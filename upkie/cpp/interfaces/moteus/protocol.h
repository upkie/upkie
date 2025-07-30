// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 *     SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <tuple>

#include "upkie/cpp/interfaces/moteus/Data.h"
#include "upkie/cpp/interfaces/moteus/Mode.h"
#include "upkie/cpp/interfaces/moteus/Output.h"
#include "upkie/cpp/interfaces/moteus/PositionCommand.h"
#include "upkie/cpp/interfaces/moteus/PositionResolution.h"
#include "upkie/cpp/interfaces/moteus/QueryResult.h"
#include "upkie/cpp/interfaces/moteus/Resolution.h"
#include "upkie/cpp/interfaces/moteus/ServoCommand.h"
#include "upkie/cpp/interfaces/moteus/ServoReply.h"

/* @file
 *
 * This describes helper classes useful for constructing and parsing
 * CAN-FD packets for the moteus brushless servo.
 */

namespace upkie::cpp::interfaces {

namespace moteus {

enum {
  kCurrentRegisterMapVersion = 4,
};

enum Multiplex : uint32_t {
  kWriteBase = 0x00,
  kWriteInt8 = 0x00,
  kWriteInt16 = 0x04,
  kWriteInt32 = 0x08,
  kWriteFloat = 0x0c,

  kReadBase = 0x10,
  kReadInt8 = 0x10,
  kReadInt16 = 0x14,
  kReadInt32 = 0x18,
  kReadFloat = 0x1c,

  kReplyBase = 0x20,
  kReplyInt8 = 0x20,
  kReplyInt16 = 0x24,
  kReplyInt32 = 0x28,
  kReplyFloat = 0x2c,

  kWriteError = 0x30,
  kReadError = 0x31,

  // # Tunneled Stream #
  kClientToServer = 0x40,
  kServerToClient = 0x41,
  kClientPollServer = 0x42,

  kNop = 0x50,
};

enum Register : uint32_t {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kRezeroState = 0x00c,
  kVoltage = 0x00d,
  kTemperature = 0x00e,
  kFault = 0x00f,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,
  kVoltageDqD = 0x01a,
  kVoltageDqQ = 0x01b,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxTorque = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,

  kPositionKp = 0x030,
  kPositionKi = 0x031,
  kPositionKd = 0x032,
  kPositionFeedforward = 0x033,
  kPositionCommand = 0x034,

  kRegisterMapVersion = 0x102,
  kSerialNumber = 0x120,
  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,

  kRezero = 0x130,
};

template <typename T>
T Saturate(double value, double scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();

  const double double_max = static_cast<T>(max);
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  if (scaled < -double_max) {
    return -max;
  }
  if (scaled > double_max) {
    return max;
  }
  return static_cast<T>(scaled);
}

struct CanFrame {
  uint8_t data[64] = {};
  uint8_t size = 0;
};

class WriteCanFrame {
 public:
  explicit WriteCanFrame(CanFrame* frame)
      : data_(&frame->data[0]), size_(&frame->size) {}

  WriteCanFrame(uint8_t* data, uint8_t* size) : data_(data), size_(size) {}

  template <typename T, typename X>
  void Write(X value_in) {
    T value = static_cast<T>(value_in);
    if (sizeof(value) + *size_ > 64) {
      throw std::runtime_error("overflow");
    }

#ifndef __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures supported"
#endif
    std::memcpy(&data_[*size_], reinterpret_cast<const char*>(&value),
                sizeof(value));
    *size_ += sizeof(value);
  }

  void WriteMapped(double value, double int8_scale, double int16_scale,
                   double int32_scale, Resolution res) {
    switch (res) {
      case Resolution::kInt8: {
        Write<int8_t>(Saturate<int8_t>(value, int8_scale));
        break;
      }
      case Resolution::kInt16: {
        Write<int16_t>(Saturate<int16_t>(value, int16_scale));
        break;
      }
      case Resolution::kInt32: {
        Write<int32_t>(Saturate<int32_t>(value, int32_scale));
        break;
      }
      case Resolution::kFloat: {
        Write<float>(static_cast<float>(value));
        break;
      }
      case Resolution::kIgnore: {
        throw std::runtime_error("Attempt to write ignored resolution");
      }
    }
  }

  void WritePosition(double value, Resolution res) {
    WriteMapped(value, 0.01, 0.0001, 0.00001, res);
  }

  void WriteVelocity(double value, Resolution res) {
    WriteMapped(value, 0.1, 0.00025, 0.00001, res);
  }

  void WriteTorque(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.01, 0.001, res);
  }

  void WritePwm(double value, Resolution res) {
    WriteMapped(value, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0, res);
  }

  void WriteVoltage(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.1, 0.001, res);
  }

  void WriteTemperature(float value, Resolution res) {
    WriteMapped(value, 1.0, 0.1, 0.001, res);
  }

  void WriteTime(float value, Resolution res) {
    WriteMapped(value, 0.01, 0.001, 0.000001, res);
  }

 private:
  uint8_t* const data_;
  uint8_t* const size_;
};

/// Determines how to group registers when encoding them to minimize
/// the required bytes.
template <size_t N>
class WriteCombiner {
 public:
  template <typename T>
  WriteCombiner(WriteCanFrame* frame, int8_t base_command, T start_register,
                std::array<Resolution, N> resolutions)
      : frame_(frame),
        base_command_(base_command),
        start_register_(start_register),
        resolutions_(resolutions) {}

  ~WriteCombiner() {
    if (offset_ != N) {
      ::abort();
    }
  }

  bool MaybeWrite() {
    const auto this_offset = offset_;
    offset_++;

    if (current_resolution_ == resolutions_[this_offset]) {
      // We don't need to write any register operations here, and the
      // value should go out only if requested.
      return current_resolution_ != Resolution::kIgnore;
    }
    // We need to do some kind of framing.  See how far ahead the new
    // resolution goes.
    const auto new_resolution = resolutions_[this_offset];
    current_resolution_ = new_resolution;

    // We are now in a new block of ignores.
    if (new_resolution == Resolution::kIgnore) {
      return false;
    }

    int count = 1;
    for (size_t i = this_offset + 1; i < N && resolutions_[i] == new_resolution;
         i++) {
      count++;
    }

    int8_t write_command = base_command_ + [&]() {
      switch (new_resolution) {
        case Resolution::kInt8:
          return 0x00;
        case Resolution::kInt16:
          return 0x04;
        case Resolution::kInt32:
          return 0x08;
        case Resolution::kFloat:
          return 0x0c;
        case Resolution::kIgnore: {
          throw std::logic_error("unreachable");
        }
      }
      return 0x00;
    }();

    if (count <= 3) {
      // Use the shorthand formulation.
      frame_->Write<int8_t>(write_command + count);
    } else {
      // Nope, the long form.
      frame_->Write<int8_t>(write_command);
      frame_->Write<int8_t>(count);
    }
    if ((start_register_ + this_offset) > 127) {
      throw std::logic_error("unsupported");
    }
    frame_->Write<int8_t>(start_register_ + this_offset);
    return true;
  }

 private:
  WriteCanFrame* const frame_;
  int8_t base_command_;
  uint32_t start_register_;
  std::array<Resolution, N> resolutions_;

  Resolution current_resolution_ = Resolution::kIgnore;
  size_t offset_ = 0;
};

class MultiplexParser {
 public:
  explicit MultiplexParser(const CanFrame* frame)
      : data_(&frame->data[0]), size_(frame->size) {}

  MultiplexParser(const uint8_t* data, uint8_t size)
      : data_(data), size_(size) {}

  std::tuple<bool, uint32_t, Resolution> next() {
    if (offset_ >= size_) {
      // We are done.
      return std::make_tuple(false, 0, Resolution::kInt8);
    }

    if (remaining_) {
      remaining_--;
      const auto this_register = current_register_++;

      // Do we actually have enough data?
      if (offset_ + ResolutionSize(current_resolution_) > size_) {
        return std::make_tuple(false, 0, Resolution::kInt8);
      }

      return std::make_tuple(true, this_register, current_resolution_);
    }

    // We need to look for another command.
    while (offset_ < size_) {
      const auto cmd = data_[offset_++];
      if (cmd == Multiplex::kNop) {
        continue;
      }

      // We are guaranteed to still need data.
      if (offset_ >= size_) {
        // Nope, we are out.
        break;
      }

      if (cmd >= 0x20 && cmd < 0x30) {
        // This is a regular reply of some sort.
        const auto id = (cmd >> 2) & 0x03;
        current_resolution_ = [id]() {
          switch (id) {
            case 0:
              return Resolution::kInt8;
            case 1:
              return Resolution::kInt16;
            case 2:
              return Resolution::kInt32;
            case 3:
              return Resolution::kFloat;
          }
          // we cannot reach this point
          return Resolution::kInt8;
        }();
        int count = cmd & 0x03;
        if (count == 0) {
          count = data_[offset_++];

          // We still need more data.
          if (offset_ >= size_) {
            break;
          }
        }

        if (count == 0) {
          // Empty, guess we can ignore.
          continue;
        }

        current_register_ = data_[offset_++];
        remaining_ = count - 1;

        if (offset_ + ResolutionSize(current_resolution_) > size_) {
          return std::make_tuple(false, 0, Resolution::kInt8);
        }

        return std::make_tuple(true, current_register_++, current_resolution_);
      }

      // For anything else, we'll just assume it is an error of some
      // sort and stop parsing.
      offset_ = size_;
      break;
    }
    return std::make_tuple(false, 0, Resolution::kInt8);
  }

  template <typename T>
  T Read() {
    if (offset_ + sizeof(T) > size_) {
      throw std::runtime_error("overrun");
    }

    T result = {};
    std::memcpy(&result, &data_[offset_], sizeof(T));
    offset_ += sizeof(T);
    return result;
  }

  template <typename T>
  double Nanify(T value) {
    if (value == std::numeric_limits<T>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value;
  }

  double ReadMapped(Resolution res, double int8_scale, double int16_scale,
                    double int32_scale) {
    switch (res) {
      case Resolution::kInt8: {
        return Nanify<int8_t>(Read<int8_t>()) * int8_scale;
      }
      case Resolution::kInt16: {
        return Nanify<int16_t>(Read<int16_t>()) * int16_scale;
      }
      case Resolution::kInt32: {
        return Nanify<int32_t>(Read<int32_t>()) * int32_scale;
      }
      case Resolution::kFloat: {
        return Read<float>();
      }
      default: {
        break;
      }
    }
    throw std::logic_error("unreachable");
  }

  int ReadInt(Resolution res) {
    return static_cast<int>(ReadMapped(res, 1.0, 1.0, 1.0));
  }

  double ReadPosition(Resolution res) {
    return ReadMapped(res, 0.01, 0.0001, 0.00001);
  }

  double ReadVelocity(Resolution res) {
    return ReadMapped(res, 0.1, 0.00025, 0.00001);
  }

  double ReadTorque(Resolution res) {
    return ReadMapped(res, 0.5, 0.01, 0.001);
  }

  double ReadPwm(Resolution res) {
    return ReadMapped(res, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0);
  }

  double ReadVoltage(Resolution res) {
    return ReadMapped(res, 0.5, 0.1, 0.001);
  }

  double ReadTemperature(Resolution res) {
    return ReadMapped(res, 1.0, 0.1, 0.001);
  }

  double ReadTime(Resolution res) {
    return ReadMapped(res, 0.01, 0.001, 0.000001);
  }

  double ReadCurrent(Resolution res) {
    return ReadMapped(res, 1.0, 0.1, 0.001);
  }

  void Ignore(Resolution res) { offset_ += ResolutionSize(res); }

 private:
  int ResolutionSize(Resolution res) {
    switch (res) {
      case Resolution::kInt8:
        return 1;
      case Resolution::kInt16:
        return 2;
      case Resolution::kInt32:
        return 4;
      case Resolution::kFloat:
        return 4;
      default: {
        break;
      }
    }
    return 1;
  }

  const uint8_t* const data_;
  const uint8_t size_;
  size_t offset_ = 0;

  int remaining_ = 0;
  Resolution current_resolution_ = Resolution::kIgnore;
  uint32_t current_register_ = 0;
};

inline void EmitStopCommand(WriteCanFrame* frame) {
  frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
  frame->Write<int8_t>(Register::kMode);
  frame->Write<int8_t>(Mode::kStopped);
}

inline void EmitPositionCommand(WriteCanFrame* frame,
                                const PositionCommand& command,
                                const PositionResolution& resolution) {
  // First, set the position mode.
  frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
  frame->Write<int8_t>(Register::kMode);
  frame->Write<int8_t>(Mode::kPosition);

  // Now we use some heuristics to try and group consecutive registers
  // of the same resolution together into larger writes.
  WriteCombiner<8> combiner(frame, 0x00, Register::kCommandPosition,
                            {
                                resolution.position,
                                resolution.velocity,
                                resolution.feedforward_torque,
                                resolution.kp_scale,
                                resolution.kd_scale,
                                resolution.maximum_torque,
                                resolution.stop_position,
                                resolution.watchdog_timeout,
                            });

  if (combiner.MaybeWrite()) {
    frame->WritePosition(command.position, resolution.position);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteVelocity(command.velocity, resolution.velocity);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTorque(command.feedforward_torque,
                       resolution.feedforward_torque);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePwm(command.kp_scale, resolution.kp_scale);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePwm(command.kd_scale, resolution.kd_scale);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTorque(command.maximum_torque, resolution.maximum_torque);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePosition(command.stop_position, resolution.stop_position);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTime(command.watchdog_timeout, resolution.watchdog_timeout);
  }
}

inline void EmitQueryCommand(WriteCanFrame* frame,
                             const QueryCommand& command) {
  {
    WriteCombiner<6> combiner(frame, 0x10, Register::kMode,
                              {
                                  command.mode,
                                  command.position,
                                  command.velocity,
                                  command.torque,
                                  command.q_current,
                                  command.d_current,
                              });
    for (int i = 0; i < 6; i++) {
      combiner.MaybeWrite();
    }
  }
  {
    WriteCombiner<4> combiner(frame, 0x10, Register::kRezeroState,
                              {command.rezero_state, command.voltage,
                               command.temperature, command.fault});
    for (int i = 0; i < 4; i++) {
      combiner.MaybeWrite();
    }
  }
}

inline QueryResult ParseQueryResult(const uint8_t* data, size_t size) {
  MultiplexParser parser(data, size);

  QueryResult result;
  while (true) {
    auto entry = parser.next();
    if (!std::get<0>(entry)) {
      break;
    }
    const auto res = std::get<2>(entry);
    switch (static_cast<Register>(std::get<1>(entry))) {
      case Register::kMode: {
        result.mode = static_cast<Mode>(parser.ReadInt(res));
        break;
      }
      case Register::kPosition: {
        result.position = parser.ReadPosition(res);
        break;
      }
      case Register::kVelocity: {
        result.velocity = parser.ReadVelocity(res);
        break;
      }
      case Register::kTorque: {
        result.torque = parser.ReadTorque(res);
        break;
      }
      case Register::kQCurrent: {
        result.q_current = parser.ReadCurrent(res);
        break;
      }
      case Register::kDCurrent: {
        result.d_current = parser.ReadCurrent(res);
        break;
      }
      case Register::kRezeroState: {
        result.rezero_state = parser.ReadInt(res) != 0;
        break;
      }
      case Register::kVoltage: {
        result.voltage = parser.ReadVoltage(res);
        break;
      }
      case Register::kTemperature: {
        result.temperature = parser.ReadTemperature(res);
        break;
      }
      case Register::kFault: {
        result.fault = parser.ReadInt(res);
        break;
      }
      default: {
        parser.Ignore(res);
      }
    }
  }

  return result;
}

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
