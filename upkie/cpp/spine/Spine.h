// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <mpacklog/Logger.h>

#include <algorithm>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "upkie/cpp/controllers/ControllerPipeline.h"
#include "upkie/cpp/interfaces/Interface.h"
#include "upkie/cpp/observers/ObserverPipeline.h"
#include "upkie/cpp/sensors/SensorPipeline.h"
#include "upkie/cpp/spine/AgentInterface.h"
#include "upkie/cpp/spine/StateMachine.h"
#include "upkie/cpp/utils/SynchronousClock.h"

//! Main control loop between agents and actuators.
namespace upkie::cpp::spine {

//! Number of bits in one mebibyte.
constexpr size_t kMebibytes = 1 << 20;

/*! Loop transmitting actions to the actuation and observations to the agent.
 *
 * The spine acts as the intermediary between the actuation interface (e.g.
 * moteus servos connected to the CAN-FD bus) and an agent communicating over
 * shared memory (the agent interface). It packs observations to the agent from
 * actuation replies and commands to the actuation from agent actions.
 *
 * The spine processes requests at the beginning and end of each control cycle
 * according to its StateMachine. The overall specification of the state
 * machine is summarized in the following diagram:
 *
 * \image html state-machine.png
 * \image latex state-machine.eps
 *
 * See StateMachine for more details.
 */
class Spine {
  using ControllerPipeline = upkie::cpp::controllers::ControllerPipeline;
  using ObserverPipeline = upkie::cpp::observers::ObserverPipeline;
  using Output = upkie::cpp::interfaces::moteus::Output;
  using SensorPipeline = upkie::cpp::sensors::SensorPipeline;
  using ServoReply = upkie::cpp::interfaces::moteus::ServoReply;

 public:
  //! Spine parameters.
  struct Parameters {
    //! CPUID for the spine thread (-1 to disable realtime).
    int cpu = -1;

    //! Frequency of the spine loop in [Hz].
    unsigned frequency = 1000u;

    //! Path to output log file
    std::string log_path = "/dev/null";

    //! Name of the shared memory object for inter-process communication
    std::string shm_name = "/upkie";

    //! Size of the shared memory object in bytes
    size_t shm_size = 1 * kMebibytes;
  };

  /*! Initialize spine.
   *
   * \param[in] params Spine parameters.
   * \param[in, out] interface Interface to actuators.
   * \param[in, out] sensors Pipeline of sensors to read from.
   * \param[in, out] observers Pipeline of observers to run, in order, at each
   *     cycle of the spine loop.
   * \param[in, out] controllers Pipeline of controllers to run, in order, at
   *     each cycle of the spine loop.
   */
  Spine(const Parameters& params, interfaces::Interface& interface,
        SensorPipeline& sensors, ObserverPipeline& observers,
        ControllerPipeline& controllers);

  /*! Reset the spine with a new configuration.
   *
   * \param[in] config New configuration dictionary forwarded to spine modules
   *     (actuation interface, observers).
   */
  void reset(const palimpsest::Dictionary& config);

  /*! Run the spine loop until termination.
   *
   * Each iteration of the loop runs observers, computes the action and cycles
   * the actuation interface. Additionally, this function collects debug values
   * and logs everything.
   *
   * \note The spine will catch keyboard interrupts once this function is
   * called.
   */
  void run();

  //! Spin one cycle of the spine loop.
  void cycle();

  /*! Alternative to \ref run where the actuation interface is cycled a fixed
   * number of times, and communication cycles are not frequency-regulated.
   *
   * \param[in] nb_substeps Number of actuation cycles per action.
   *
   * Thus function assumes the agent alternates acting and observing.
   * Simulation steps are triggered at startup (to construct the initial
   * observation) and at each action request.
   *
   * \note As its name suggests, do not use this function on a real robot.
   *
   * Note that there is currently a delay of three substeps between observation
   * and simulation. That is, the internal simulation state is always three
   * substeps ahead compared to the values written to the observation
   * dictionary when cycling the actuation. This decision is discussed in
   * https://github.com/orgs/upkie/discussions/238#discussioncomment-8984290
   */
  void simulate(unsigned nb_substeps);

  /*! Spin one cycle of communications with the actuation interface.
   *
   * A cycle consists in:
   *
   * 1. Run sensors and observers over the latest reply
   * 2. Prepare servo commands
   * 3. Wait to receive the reply from the previous cycle over the interface
   * 4. If applicable, start the next cycle:
   */
  void cycle_actuation();

 private:
  //! Begin cycle: check interrupts and read agent inputs
  void begin_cycle();

  //! End cycle: write agent outputs, apply state machine transition
  void end_cycle();

  /*! Log internal working dictionary.
   *
   * \param[in] clock Spine loop clock, if running in real time.
   */
  void log_working_dict(
      std::optional<std::reference_wrapper<utils::SynchronousClock>> clock =
          std::nullopt);

 protected:
  //! Frequency of the spine loop in [Hz].
  const unsigned frequency_;

  /*! Interface that communicates with actuators.
   *
   * The actuation interface communicates over the CAN-FD bus on real robots.
   * Otherwise, it can be for instance a mock or a simulator interface.
   */
  interfaces::Interface& actuation_;

  //! Shared memory mapping for inter-process communication.
  AgentInterface agent_interface_;

  //! Future used to wait for moteus replies.
  std::future<Output> actuation_output_;

  //! Latest servo replies. They are copied and thread-safe.
  std::vector<ServoReply> servo_replies_;

  //! All data from observation to action goes to this dictionary.
  palimpsest::Dictionary working_dict_;

  //! Pipeline of sensors, executed in that order.
  SensorPipeline sensors_;

  //! Pipeline of observers, executed in that order.
  ObserverPipeline observers_;

  //! Logger for the \ref working_dict_ produced at each cycle.
  mpacklog::Logger logger_;

  //! Buffer used to serialize/deserialize dictionaries in IPC.
  std::vector<char> ipc_buffer_;

  //! Boolean flag that becomes true when an interruption is caught.
  const bool& caught_interrupt_;

  //! Internal state machine.
  StateMachine state_machine_;

  //! State after the last Event::kCycleBeginning.
  State state_cycle_beginning_;

  //! State after the last Event::kCycleEnd.
  State state_cycle_end_;

  //! Number of actuation replies received during the latest cycle.
  size_t rx_count_;

  //! Pipeline of controllers, executed in that order.
  ControllerPipeline controllers_;
};

}  // namespace upkie::cpp::spine
