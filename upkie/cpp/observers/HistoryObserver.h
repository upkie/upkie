// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "upkie/cpp/exceptions/TypeError.h"
#include "upkie/cpp/observers/Observer.h"

namespace upkie::cpp::observers {

using exceptions::TypeError;
using palimpsest::Dictionary;

/*! Report high-frequency history vectors to lower-frequency agents.
 *
 * This observer allows processing higher-frequency signals from the spine as
 * vectors of observations reported to lower-frequency agents.
 */
template <typename T>
class HistoryObserver : public Observer {
 public:
  /*! Initialize observer.
   *
   * \param[in] keys List of keys to read values from in input observations.
   * \param[in] size Size of the history vector.
   * \param[in] default_value Value to initialize history vectors.
   */
  HistoryObserver(const std::vector<std::string>& keys, size_t size,
                  const T& default_value)
      : keys_(keys), values_(size, default_value) {}

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "history"; }

  /*! Reset observer.
   *
   * \param[in] config Configuration dictionary.
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
  //! Concatenate keys into a single string for reporting.
  std::string concatenate_keys() {
    std::string output;
    for (const auto& key : keys_) {
      output += "/" + key;
    }
    return output;
  }

  /*! Read value from an input dictionary.
   *
   * \param[in] dict Input dictionary.
   * \param[in] key_index Index we are at in the list of keys.
   */
  void read_value(const Dictionary& dict, unsigned key_index) {
    if (key_index < keys_.size()) {
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

  /*! Write values to an output dictionary.
   *
   * \param[out] dict Output dictionary.
   * \param[in] key_index Index we are at in the list of keys.
   */
  void write_values(Dictionary& dict, unsigned key_index) {
    if (key_index < keys_.size()) {
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

}  // namespace upkie::cpp::observers
