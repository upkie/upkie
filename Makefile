# Makefile for upkie targets
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

# Project name needs to match the one in WORKSPACE
PROJECT_NAME = upkie

BAZEL = $(CURDIR)/tools/bazelisk
COVERAGE_DIR = $(CURDIR)/bazel-out/_coverage
CURDATE = $(shell date -Iseconds)
CURDIR_NAME = $(shell basename $(CURDIR))
RASPUNZEL = $(CURDIR)/tools/raspunzel

.PHONY: deprecation_warning
deprecation_warning:
	@echo "Deprecation warning: this legacy Makefile will be removed in upkie v10."
	@echo "Try replacing \`make <task>\` by \`pixi run <task>\`\n"
.DEFAULT_GOAL := deprecation_warning

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help: deprecation_warning
	@echo "Host targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRaspberry Pi targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo ""  # manicure

# HOST TARGETS
# ============

.PHONY: build
build: deprecation_warning clean_broken_links  ## build Raspberry Pi targets
	$(BAZEL) build --config=pi64 //spines:mock_spine
	$(BAZEL) build --config=pi64 //spines:pi3hat_spine

.PHONY: check_upkie_name
check_upkie_name: deprecation_warning
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
clean: deprecation_warning clean_broken_links  ## clean all local build and intermediate files
	$(BAZEL) clean --expunge
	find $(CURDIR) -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	rm -rf $(CURDIR)/cache $(CURDIR)/dist

.PHONY: clean_broken_links
clean_broken_links: deprecation_warning
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete 2>/dev/null || true

pack_pixi_env: deprecation_warning  ## pack pixi environment to environment.tar
	pixi run pack

# This rule is handy if the target Upkie is not connected to the Internet
.PHONY: set_date
set_date: deprecation_warning check_upkie_name
	ssh ${UPKIE_NAME} sudo date -s "$(CURDATE)"

# Running `raspunzel -s` can create __pycache__ directories owned by root
# that rsync is not allowed to remove. We therefore give permissions first.
.PHONY: upload
upload: deprecation_warning check_upkie_name build set_date  ## upload built targets to the Raspberry Pi
	ssh ${UPKIE_NAME} mkdir -p $(PROJECT_NAME)
	ssh ${UPKIE_NAME} sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded \
		--exclude .mypy_cache/ \
		--exclude .pixi \
		--exclude __pycache__ \
		--exclude bazel-$(CURDIR_NAME) \
		--exclude bazel-$(PROJECT_NAME)/ \
		--exclude bazel-out/ \
		--exclude bazel-testlogs/ \
		--exclude cache/ \
		--exclude tools/bazel \
		--exclude tools/logs/\*.mpack \
		--exclude tools/raspios \
		--progress $(CURDIR)/ ${UPKIE_NAME}:$(PROJECT_NAME)/

# REMOTE TARGETS
# ==============

run_agent: deprecation_warning  ### run agent
	@if [ -f $(CURDIR)/activate.sh ]; then \
		echo "Running agent from packed environment..."; \
		. $(CURDIR)/activate.sh && python agent/main.py; \
	else \
		echo "Running agent directly..."; \
		python agent/main.py; \
	fi

run_mock_spine: deprecation_warning  ### run the mock spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:mock_spine

# NB: run_pi3hat_spine is used in build instructions
run_pi3hat_spine: deprecation_warning  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine

unpack_pixi_env: deprecation_warning  ### unpack Python environment
	@pixi-unpack environment.tar -e upkie -o ${MAMBA_ROOT_PREFIX}/envs || { \
		echo "Error: pixi-pack not found"; \
		echo "See https://github.com/Quantco/pixi-pack?tab=readme-ov-file#-installation"; \
		exit 1; \
	}
