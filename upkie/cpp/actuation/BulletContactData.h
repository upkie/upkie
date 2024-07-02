// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <string>

namespace upkie {

/*! Contact information for a single link.
 *
 * This is a placeholder class that can grow later if we want to compute a net
 * wrench from individual contact forces reported by Bullet.
 */
struct BulletContactData {
  //! Number of contact points detected on link
  int num_contact_points;
};

}  // namespace upkie
