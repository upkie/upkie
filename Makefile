# Makefile for upkie targets
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

# Hostname or IP address of the Raspberry Pi Uses the value from the UPKIE_NAME
# environment variable, if defined. Valid usage: ``make upload UPKIE_NAME=foo``
REMOTE = ${UPKIE_NAME}

# Project name needs to match the one in WORKSPACE
PROJECT_NAME = upkie

BAZEL = $(CURDIR)/tools/bazelisk
COVERAGE_DIR = $(CURDIR)/bazel-out/_coverage
CURDATE = $(shell date -Iseconds)
CURDIR_NAME = $(shell basename $(CURDIR))
RASPUNZEL = $(CURDIR)/tools/raspunzel

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Host targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRaspberry Pi targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo ""  # manicure
.DEFAULT_GOAL := help

# HOST TARGETS
# ============

.PHONY: build
build: clean_broken_links  ## build Raspberry Pi targets
	$(BAZEL) build --config=pi64 //pid_balancer
	$(BAZEL) build --config=pi64 //spines:mock_spine
	$(BAZEL) build --config=pi64 //spines:pi3hat_spine

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
clean: clean_broken_links  ## clean all local build and intermediate files
	$(BAZEL) clean --expunge

.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete

.PHONY: coverage
coverage:  # check unit test coverage and open an HTML report in Firefox (not documented in `make help`)
	$(BAZEL) coverage --combined_report=lcov --compilation_mode=fastbuild --instrument_test_targets //...
	@if [ -z "$(shell which genhtml)" ]; then\
		echo "Error: genhtml not found, is lcov installed?"; \
	else \
		genhtml $(COVERAGE_DIR)/_coverage_report.dat -o $(COVERAGE_DIR); \
		firefox $(COVERAGE_DIR)/index.html; \
	fi

.PHONY: run_bullet_spine
run_bullet_spine:  ## run the Bullet simulation spine
	$(BAZEL) run //spines:bullet_spine -- --show

# This rule is handy if the target Upkie is not connected to the Internet
.PHONY: set_date
set_date:
	ssh $(REMOTE) sudo date -s "$(CURDATE)"

# Running ``raspunzel -s`` can create __pycache__ directories owned by root
# that rsync is not allowed to remove. We therefore give permissions first.
.PHONY: upload
upload: check_upkie_name build set_date  ## upload built targets to the Raspberry Pi
	ssh $(REMOTE) mkdir -p $(PROJECT_NAME)
	ssh $(REMOTE) sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded \
		--exclude __pycache__ \
		--exclude bazel-$(CURDIR_NAME) \
		--exclude bazel-$(PROJECT_NAME)/ \
		--exclude bazel-out/ \
		--exclude bazel-testlogs/ \
		--exclude cache/ \
		--exclude tools/bazel \
		--exclude tools/logs/\*.mpack \
		--exclude tools/raspios/.packer_\* \
		--exclude tools/raspios/\*.img \
		--progress $(CURDIR)/ $(REMOTE):$(PROJECT_NAME)/

# REMOTE TARGETS
# ==============

run_mock_spine:  ### run the mock spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:mock_spine

# NB: run_pi3hat_spine is used in build instructions
run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine

# NB: run_pid_balancer is used in build instructions
run_pid_balancer:  ### run the test balancer on the Raspberry Pi
	$(RASPUNZEL) run -s //pid_balancer:pid_balancer
