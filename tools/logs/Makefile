# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0
#
# Convenience Makefile to synchronize Upkie log files
#
# Put this Makefile in the directory to synchronize and run `make download` to
# download the latest logs from the robot.

# Customize remote path to taste
REMOTE_PATH = logs

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Rules:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

.PHONY: check_upkie_name
check_upkie_name:
	@if [ -z "${UPKIE_NAME}" ]; then \
		echo "ERROR: Environment variable UPKIE_NAME is not set.\n"; \
		echo "This variable should contain the robot's hostname or IP address for SSH. "; \
		echo "You can define it inline for a one-time use:\n"; \
		echo "    make some_target UPKIE_NAME=your_robot_hostname\n"; \
		echo "Or add the following line to your shell configuration:\n"; \
		echo "    export UPKIE_NAME=your_robot_hostname\n"; \
		exit 1; \
	fi

.PHONY: clean
clean:  ## Clean up logs smaller than 100 KiB
	find . -name '*.mpack' -size "-100k" -delete

.PHONY: compress
compress:  ## Compress logs
	@for i in *.mpack; do \
		echo "Compressing $$i..."; \
		tar jcf $$i.tar.bz2 $$i; \
		rm -f $$i; \
	done

.PHONY: foxplot_last
foxplot_last:  ## Plot latest log
	foxplot $(shell ls *.mpack | tail -n 1)

.PHONY: pp
pp: pull foxplot_last  ## Shorthand for `pull` then `foxplot_last`

.PHONY: pull
pull: check_upkie_name  ## Download logs and remove them from the remote directory afterwards
	rsync -auvz --progress --exclude Makefile ${UPKIE_NAME}:$(REMOTE_PATH)/ $(CURDIR)/
	-ssh ${UPKIE_NAME} rm -f $(REMOTE_PATH)/\*.mpack
