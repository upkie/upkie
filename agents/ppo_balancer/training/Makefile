# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0
#
# Convenience Makefile to organize ppo_balancer training directories.
#
# Put this Makefile in the training directory configured by the
# UPKIE_TRAINING_PATH environment variable. Run ``make tensorboard`` to start a
# new TensorBoard instance with today's training data.

DATE=$(shell date +%Y-%m-%d)

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Rules:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

tensorboard:  ## Start tensorboard on today's trainings
	rm -f $(CURDIR)/today
	ln -sf $(CURDIR)/$(DATE) $(CURDIR)/today
	firefox http://localhost:6006 &
	tensorboard --logdir $(CURDIR)/$(DATE)
