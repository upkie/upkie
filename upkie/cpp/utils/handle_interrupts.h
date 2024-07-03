// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <signal.h>

#include <cstddef>

namespace upkie::utils {

namespace internal {

//! Internal interrupt flag.
extern bool interrupt_flag;

//! Internal handler to set \ref interrupt_flag.
void handle_interrupt(int _);

}  // namespace internal

/*! Direct interrupts (e.g. Ctrl-C) to a boolean flag.
 *
 * \return Reference to a boolean flag, which is initially false and becomes
 *     true the first time an interruption is caught.
 */
const bool& handle_interrupts();

}  // namespace upkie::utils
