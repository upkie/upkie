// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "vulp/exceptions/TypeError.h"
#include "vulp/observation/Observer.h"

namespace vulp::observation {

using palimpsest::Dictionary;
using vulp::exceptions::TypeError;
using vulp::observation::Observer;

template <typename T>
class HistoryObserver : public Observer {
 public:
  /*! Initialize observer.
   *
   * \param[in] default_value Value to initialize history vectors.
   * \param[in] prefix Dictionary key where outputs will be written.
   */
  HistoryObserver(const std::vector<std::string>& keys, size_t size,
                  const T& default_value)
      : keys_(keys), values_(size, default_value) {}

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "history"; }

  /*! Reset observer.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config) final {}

  /*! Read inputs from other observations.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final {
    return read_value(observation, 0);
  }

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final {
    auto& output = observation(prefix());
    write_values(output, 0);
  }

 private:
  std::string concatenate_keys() {
    std::string output;
    for (const auto& key : keys_) {
      output += "/" + key;
    }
    return output;
  }

  void read_value(const Dictionary& dict, unsigned key_index) {
    if (key_index != keys_.size()) {
      const std::string& key = keys_.at(key_index);
      return read_value(dict(key), key_index + 1);
    }
    if (!dict.is_value()) {
      throw TypeError(
          __FILE__, __LINE__,
          "Observation at " + concatenate_keys() + " is not a value");
    }
    const T& new_value = dict.as<T>();
    values_.pop_back();
    values_.insert(values_.begin(), new_value);
  }

  void write_values(Dictionary& dict, unsigned key_index) {
    if (key_index != keys_.size()) {
      const std::string& key = keys_.at(key_index);
      return write_values(dict(key), key_index + 1);
    }
    if (dict.is_empty()) {
      dict = values_;
    } else {
      dict.as<std::vector<T>>() = values_;
    }
  }

 private:
  //! Keys locating values to read from in input observations.
  std::vector<std::string> keys_;

  //! History of recent values written to output observations.
  std::vector<T> values_;
};

}  // namespace vulp::observation
