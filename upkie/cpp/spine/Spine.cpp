// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     SPDX-License-Identifier: Apache-2.0
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 */

#include "upkie/cpp/spine/Spine.h"

#include <mpacklog/Logger.h>

#include <limits>

#include "upkie/cpp/exceptions/ObserverError.h"
#include "upkie/cpp/observers/observe_servos.h"
#include "upkie/cpp/observers/observe_time.h"
#include "upkie/cpp/utils/handle_interrupts.h"
#include "upkie/cpp/utils/realtime.h"

namespace upkie::cpp::spine {

using palimpsest::Dictionary;
using upkie::cpp::actuation::moteus::Output;

Spine::Spine(const Parameters& params, actuation::Interface& actuation,
             ObserverPipeline& observers)
    : frequency_(params.frequency),
      actuation_(actuation),
      agent_interface_(params.shm_name, params.shm_size),
      observer_pipeline_(observers),
      logger_(params.log_path),
      caught_interrupt_(utils::handle_interrupts()),
      state_machine_(agent_interface_),
      state_cycle_beginning_(State::kOver),
      state_cycle_end_(State::kOver),
      rx_count_(0) {
// Thread name as it appears in the `cmd` column of `ps`
#ifdef __APPLE__
  pthread_setname_np("spine_thread");
#else
  pthread_setname_np(pthread_self(), "spine_thread");
#endif

  // Real-time configuration
  // NB: it is too late to lock memory here, this should be done by the caller
  if (params.cpu >= 0) {
    utils::configure_cpu(params.cpu);
    utils::configure_scheduler(10);
  }

  // Inter-process communication
  agent_interface_.set_request(Request::kNone);

  // Initialize internal dictionary
  Dictionary& observation = working_dict_("observation");
  observers::observe_time(observation);
  working_dict_.insert<double>("time", observation.get<double>("time"));
}

void Spine::reset(const Dictionary& config) {
  Dictionary& action = working_dict_("action");
  actuation_.reset(config);
  action.clear();
  actuation_.reset_action(action);
  observer_pipeline_.reset(config);
  spdlog::info("Spine configured with:\n\n{}\n", config);
}

void Spine::log_working_dict() {
  // Log spine entries to the working dictionary
  Dictionary& spine = working_dict_("spine");
  spine("logger_last_size") = static_cast<uint32_t>(logger_.last_size());
  spine("rx_count") = static_cast<uint32_t>(rx_count_);
  spine("state_cycle_beginning") =
      static_cast<uint32_t>(state_cycle_beginning_);
  spine("state_cycle_end") = static_cast<uint32_t>(state_cycle_end_);

  // Log full working dictionary
  if (!logger_.put(working_dict_)) {
    spdlog::warn("Could not log spine dictionary, logger is full");
  }

  // Log configuration dictionary at most once (at reset)
  if (working_dict_.has("config")) {
    working_dict_.remove("config");
  }
}

void Spine::run() {
  Dictionary& spine = working_dict_("spine");
  utils::SynchronousClock clock(frequency_);
  while (state_machine_.state() != State::kOver) {
    cycle();
    if (state_machine_.state() != State::kSendStops) {
      spine("clock")("measured_period") = clock.measured_period();
      spine("clock")("skip_count") = clock.skip_count();
      spine("clock")("slack") = clock.slack();
      log_working_dict();
    }
    clock.wait_for_next_tick();
  }
  spdlog::info("SEE YOU SPACE COWBOY...");
}

void Spine::cycle() {
  begin_cycle();      // check interrupts, read agent inputs
  cycle_actuation();  // read latest actuation replies, send new commands
  end_cycle();        // output to agent
}

void Spine::simulate(unsigned nb_substeps) {
  while (state_machine_.state() != State::kOver) {
    begin_cycle();
    if (state_machine_.state() == State::kReset) {
      cycle_actuation();  // S1: cycle the simulator, promise actuation_output_
      cycle_actuation();  // S2: fill servo_replies_ from actuation_output_
      cycle_actuation();  // S3: fill observation dict from servo_replies_
      end_cycle();
      // now the first observation is ready to be read by the agent
    } else if (state_machine_.state() == State::kStep) {
      cycle_actuation();
      end_cycle();  // important: writes observation of the first substep
      for (unsigned substep = 1; substep < nb_substeps; ++substep) {
        cycle_actuation();
      }
    } else {
      end_cycle();
    }
  }
}

void Spine::begin_cycle() {
  if (caught_interrupt_) {
    state_machine_.process_event(Event::kInterrupt);
  } else /* (!caught_interrupt_) */ {
    state_machine_.process_event(Event::kCycleBeginning);
  }
  state_cycle_beginning_ = state_machine_.state();

  try {
    // Read input dictionary if applicable
    if (state_machine_.state() == State::kReset) {
      Dictionary& config = working_dict_("config");
      const char* data = agent_interface_.data();
      size_t size = agent_interface_.size();
      config.clear();
      config.update(data, size);
      reset(config);
    } else if (state_machine_.state() == State::kStep) {
      Dictionary& action = working_dict_("action");
      const char* data = agent_interface_.data();
      size_t size = agent_interface_.size();
      action.update(data, size);
    }
  } catch (const palimpsest::exceptions::PalimpsestError& exn) {
    spdlog::error("Deserialization error: {}", exn.what());
    state_machine_.process_event(Event::kInterrupt);
  }
}

void Spine::end_cycle() {
  // Write observation if applicable
  const Dictionary& observation = working_dict_("observation");
  working_dict_("time") = observation.get<double>("time");
  if (state_machine_.state() == State::kReset ||
      state_machine_.state() == State::kStep) {
    size_t size = observation.serialize(ipc_buffer_);
    agent_interface_.write(ipc_buffer_.data(), size);
  }

  state_machine_.process_event(Event::kCycleEnd);
  state_cycle_end_ = state_machine_.state();
}

void Spine::cycle_actuation() {
  try {
    // 1. Observation
    Dictionary& observation = working_dict_("observation");
    observers::observe_time(observation);
    observers::observe_servos(observation, actuation_.servo_name_map(),
                              servo_replies_);
    actuation_.observe(observation);
    // Observers need configuration, so they cannot run at stop
    if (state_machine_.state() != State::kSendStops &&
        state_machine_.state() != State::kShutdown) {
      try {
        observer_pipeline_.run(observation);
      } catch (const exceptions::ObserverError& e) {
        spdlog::info("Key error from {}: key \"{}\" not found", e.prefix(),
                     e.key());
      }
    }

    // 2. Action
    if (state_machine_.state() == State::kSendStops ||
        state_machine_.state() == State::kShutdown) {
      actuation_.write_stop_commands();
    } else if (state_machine_.state() == State::kStep) {
      const Dictionary& action = working_dict_("action");
      actuation_.process_action(action);
      actuation_.write_position_commands(action);
    }
  } catch (const std::exception& e) {
    spdlog::error("[Spine] Caught an exception: {}", e.what());
    spdlog::error("[Spine] Sending stop commands...");
    state_machine_.process_event(Event::kInterrupt);
    actuation_.write_stop_commands();
  } catch (...) {
    spdlog::error("[Spine] Caught an unknown exception!");
    spdlog::error("[Spine] Sending stop commands...");
    state_machine_.process_event(Event::kInterrupt);
    actuation_.write_stop_commands();
  }

  // Whatever exceptions were thrown around, we caught them and at this
  // point every actuation command is either a stop or a position one.

  // 3. Wait for the result of the last query and copy it
  if (actuation_output_.valid()) {
    const auto current_values = actuation_output_.get();  // may wait here
    rx_count_ = current_values.query_result_size;
    servo_replies_.resize(rx_count_);
    std::copy(actuation_.replies().begin(),
              actuation_.replies().begin() + rx_count_, servo_replies_.begin());
  }

  // Now we are after the previous cycle (we called actuation_output_.get())
  // and before the next one. This is a good time to break out of loop of
  // communication cycles. Otherwise, the interface may warn that it is waiting
  // for the last actuation cycle to finish.
  if (state_machine_.is_over_after_this_cycle()) {
    spdlog::info("Wrapping up last communication cycle");
    return;
  }

  // 4. Start a new cycle. Results have been copied, so actuation commands and
  // replies are available again to the actuation thread for writing.
  auto promise = std::make_shared<std::promise<Output>>();
  actuation_.cycle([promise](const Output& output) {
    // This is called from an arbitrary thread, so we
    // just set the promise value here.
    promise->set_value(output);
  });
  actuation_output_ = promise->get_future();
}

}  // namespace upkie::cpp::spine
