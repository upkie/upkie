#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

import gin


@gin.configurable
class UpkieWheelsReward:

    max_pitch: float
    max_position: float
    pitch_weight: float
    position_weight: float

    def __init__(
        self,
        lookahead_duration: float,
        max_pitch: float,
        max_position: float,
        pitch_weight: float,
        position_weight: float,
    ):
        self.lookahead_duration = lookahead_duration
        self.max_pitch = max_pitch
        self.max_position = max_position
        self.pitch_weight = pitch_weight
        self.position_weight = position_weight

    def get(self, observation):
        pitch = observation[0]
        ground_position = observation[1]
        ground_velocity = observation[2]
        angular_velocity = observation[3]

        T = self.lookahead_duration
        lookahead_pitch = pitch + T * angular_velocity
        lookahead_position = ground_position + T * ground_velocity
        normalized_lookahead_pitch = lookahead_pitch / self.max_pitch
        normalized_lookahead_position = lookahead_position / self.max_position
        return (
            1.0
            - self.pitch_weight * abs(normalized_lookahead_pitch)
            - self.position_weight * abs(normalized_lookahead_position)
        )
