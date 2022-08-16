#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def serialize(obj):
    """
    Serialize an object for logging.

    Args:
        obj: Object to serialize.

    Returns:
        Serialized object.

    Note:
        Calling the numpy conversion is much faster than the default list
        constructor:

        In [1]: x = random.random(6)

        In [2]: %timeit list(x)
        789 ns ± 5.79 ns per loop (mean ± std. dev. of 7 runs, 1e6 loops each)

        In [3]: %timeit x.tolist()
        117 ns ± 0.865 ns per loop (mean ± std. dev. of 7 runs, 1e7 loops each)
    """
    if hasattr(obj, "tolist"):  # numpy.ndarray
        return obj.tolist()
    elif hasattr(obj, "np"):  # pinocchio.SE3
        return obj.np.tolist()
    elif hasattr(obj, "serialize"):  # more complex objects
        return obj.serialize()
    return obj
