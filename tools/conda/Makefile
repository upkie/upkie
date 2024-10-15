# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria
#
# Convenience Makefile to pack/unpack an `upkie` conda environment.

PROJECT_NAME = upkie
CONDA_ENV_FILE = conda_env.tar.gz

.PHONY: help
help:
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

.PHONY: check_mamba_setup
check_mamba_setup:
	@ if [ -z "${MAMBA_EXE}" ] || [ -z "${MAMBA_ROOT_PREFIX}" ]; then \
		echo "ERROR: Either MAMBA_EXE or MAMBA_ROOT_PREFIX is not set."; \
		echo "Is Micromamba installed?"; \
		echo "See https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html"; \
		exit 1; \
	fi

.PHONY: clean
clean:  ## remove conda environment archive
	rm -f $(CONDA_ENV_FILE)

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
