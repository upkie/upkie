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
CONDA_ENV_FILE = conda_env.tar.gz

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
	rm -f $(CONDA_ENV_FILE)

.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete

.PHONY: run_bullet_spine
run_bullet_spine:  ## run the Bullet simulation spine
	$(BAZEL) run //spines:bullet_spine -- --show

# This rule is handy if the target Upkie is not connected to the Internet
.PHONY: set_date
set_date: check_upkie_name
	ssh ${UPKIE_NAME} sudo date -s "$(CURDATE)"

# Running `raspunzel -s` can create __pycache__ directories owned by root
# that rsync is not allowed to remove. We therefore give permissions first.
.PHONY: upload
upload: check_upkie_name build set_date  ## upload built targets to the Raspberry Pi
	ssh ${UPKIE_NAME} mkdir -p $(PROJECT_NAME)
	ssh ${UPKIE_NAME} sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
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
		--progress $(CURDIR)/ ${UPKIE_NAME}:$(PROJECT_NAME)/

# REMOTE TARGETS
# ==============

run_mock_spine:  ### run the mock spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:mock_spine

# NB: run_pi3hat_spine is used in build instructions
run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine

# DEV HELPERS
# ===========

.PHONY: coverage
coverage:  # check unit test coverage and open an HTML report in Firefox (not documented in `make help`)
	$(BAZEL) coverage --combined_report=lcov --compilation_mode=fastbuild --instrument_test_targets //...
	@if [ -z "$(shell which genhtml)" ]; then\
		echo "ERROR: genhtml not found, is lcov installed?"; \
	else \
		genhtml $(COVERAGE_DIR)/_coverage_report.dat -o $(COVERAGE_DIR); \
		firefox $(COVERAGE_DIR)/index.html; \
	fi

.PHONY: lint
lint:
	$(BAZEL) test --config lint //...
	ruff check $(CURDIR)/upkie

.PHONY: test
test:
	$(BAZEL) test //...

# CONDA ENV PACKING
# =================

.PHONY: check_mamba_setup
check_mamba_setup:
	@ if [ -z "${MAMBA_EXE}" ] || [ -z "${MAMBA_ROOT_PREFIX}" ]; then \
		echo "ERROR: Either MAMBA_EXE or MAMBA_ROOT_PREFIX is not set."; \
		echo "Is Micromamba installed?"; \
		echo "See https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html"; \
		exit 1; \
	fi

.PHONY: pack_conda_env
pack_conda_env: check_mamba_setup  ## prepare conda environment to install it offline on your Upkie
	${MAMBA_EXE} env create -f environment.yaml -n raspios_$(PROJECT_NAME) --platform linux-aarch64 -y
	tar -zcf $(CONDA_ENV_FILE) -C ${MAMBA_ROOT_PREFIX}/envs/raspios_$(PROJECT_NAME) .
	${MAMBA_EXE} env remove -n raspios_$(PROJECT_NAME) -y

.PHONY: check_mamba_setup unpack_conda_env
unpack_conda_env:  ### unpack conda environment to remote conda path
	-${MAMBA_EXE} env list | grep $(PROJECT_NAME) > /dev/null && ${MAMBA_EXE} env remove -n $(PROJECT_NAME) -y
	mkdir -p ${MAMBA_ROOT_PREFIX}/envs/$(PROJECT_NAME)
	tar -zxf $(CONDA_ENV_FILE) -C ${MAMBA_ROOT_PREFIX}/envs/$(PROJECT_NAME)
