// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "vulp/observation/Observer.h"

namespace vulp::observation {

using palimpsest::Dictionary;
using vulp::observation::Observer;

template <typename T>
class HistoryObserver : public Observer {
 public:
  /*! Initialize observer.
   *
   * \param[in] default_value Value to initialize history vectors.
   * \param[in] prefix Dictionary key where outputs will be written.
   */
  explicit HistoryObserver(const T& default_value,
                           const std::string& prefix = "")
      : default_value_(default_value),
        prefix_((prefix.size() > 1) ? prefix : "history_observer") {}

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return prefix_; }

  /*! Reset observer.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config) override {
    if (!config.has(prefix_)) {
      spdlog::debug("No \"{}\" runtime configuration", prefix_);
      return;
    }

    spdlog::info("Applying \"{}\" runtime configuration", prefix_);
    values_.clear();
    reset_values(config(prefix_), values_);
  }

  void reset_values(const Dictionary& source, Dictionary& dest) {
    for (const std::string& key : source.keys()) {
      const Dictionary& child = source(key);
      if (child.is_value()) {
        const unsigned size = child.as<unsigned>();
        dest.insert<std::vector<T>>(key, size, default_value_);
      } else {
        reset_values(child, dest(key));
      }
    }
  }

  /*! Read inputs from other observations.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final {
    // ...
  }

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final {
    // ...
  }

 private:
  //! Prefix of outputs in the observation dictionary.
  std::string prefix_;

  T default_value_;

  Dictionary values_;
};

}  // namespace vulp::observation
