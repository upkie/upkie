#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria


def serialize(obj):
    """!
    Serialize an object for message packing.

    @param obj Object to serialize.

    @returns Serialized object.

    @note Calling the numpy conversion is much faster than the default list
    constructor:

    @code{python}
    In [1]: x = random.random(6)

    In [2]: %timeit list(x)
    789 ns ± 5.79 ns per loop (mean ± std. dev. of 7 runs, 1e6 loops each)

    In [3]: %timeit x.tolist()
    117 ns ± 0.865 ns per loop (mean ± std. dev. of 7 runs, 1e7 loops each)
    @endcode
    """
    if hasattr(obj, "tolist"):  # numpy.ndarray
        return obj.tolist()
    elif hasattr(obj, "np"):  # pinocchio.SE3
        return obj.np.tolist()
    elif hasattr(obj, "serialize"):  # more complex objects
        return obj.serialize()
    return obj
