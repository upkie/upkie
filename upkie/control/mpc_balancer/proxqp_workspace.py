#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import qpsolvers
from proxsuite import proxqp
from qpmpc import MPCQP


class ProxQPWorkspace:
    """Workspace data for ProxQP."""

    def __init__(
        self,
        mpc_qp: MPCQP,
        update_preconditioner: bool = True,
        verbose: bool = False,
    ):
        """Initialize the workspace.

        Args:
            mpc_qp: Model predictive control problem.
            update_preconditioner: If set, ask ProxQP to update preconditioners
                at each solve.
            verbose: If set, print out debug information.
        """
        n_eq = 0
        n_in = mpc_qp.h.size // 2  # WheeledInvertedPendulum structure
        n = mpc_qp.P.shape[1]
        solver = proxqp.dense.QP(
            n,
            n_eq,
            n_in,
            dense_backend=proxqp.dense.DenseBackend.PrimalDualLDLT,
        )
        solver.settings.eps_abs = 1e-3
        solver.settings.eps_rel = 0.0
        solver.settings.verbose = verbose
        solver.settings.compute_timings = True
        solver.settings.primal_infeasibility_solving = True
        solver.init(
            H=mpc_qp.P,
            g=mpc_qp.q,
            C=mpc_qp.G[::2, :],  # WheeledInvertedPendulum structure
            l=-mpc_qp.h[1::2],  # WheeledInvertedPendulum structure
            u=mpc_qp.h[::2],  # WheeledInvertedPendulum structure
        )
        solver.solve()
        self.update_preconditioner = update_preconditioner
        self.solver = solver

    def solve(self, mpc_qp: MPCQP) -> qpsolvers.Solution:
        """Solve a given MPC QP.

        Args:
            mpc_qp: Model predictive control problem to solve.

        Returns:
            Solution found by the solver.
        """
        self.solver.update(
            g=mpc_qp.q,
            update_preconditioner=self.update_preconditioner,
        )
        self.solver.solve()
        result = self.solver.results
        qpsol = qpsolvers.Solution(mpc_qp.problem)
        qpsol.found = result.info.status == proxqp.QPSolverOutput.PROXQP_SOLVED
        qpsol.x = self.solver.results.x
        return qpsol
