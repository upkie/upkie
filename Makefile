# Makefile for upkie targets
#
# SPDX-License-Identifier: Apache-2.0

# Project name needs to match the one in WORKSPACE
PROJECT_NAME = upkie

# Configure a Host with this name in your ~/.ssh/config
UPKIE_HOST = upkie

BAZEL = $(CURDIR)/tools/bazelisk
COVERAGE_DIR = $(CURDIR)/bazel-out/_coverage
CURDATE = $(shell date -Iseconds)
CURDIR_NAME = $(shell basename $(CURDIR))
PYTHON = python3
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

# HOST TARGETS
# ============

.PHONY: build
build: clean_broken_links  ## build Raspberry Pi targets
	$(BAZEL) build --config=pi64 //spines:pi3hat_spine

.PHONY: clean
clean: clean_broken_links  ## clean all local build and intermediate files
	$(BAZEL) clean --expunge
	find $(CURDIR) -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	rm -rf $(CURDIR)/cache $(CURDIR)/dist

.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete 2>/dev/null || true

.PHONY: run_bullet_spine
run_bullet_spine:  ## run the Bullet spine with an Upkie biped
	$(BAZEL) run //spines:bullet_spine -- --show

.PHONY: run_bullet_spine_cookie
run_bullet_spine_cookie:  ## run the Bullet spine with a Cookie biped
	$(BAZEL) run //spines:bullet_spine -- --show --robot-variant cookie

# Running `raspunzel -s` can create __pycache__ directories owned by root
# that rsync is not allowed to remove. We therefore give permissions first.
.PHONY: upload
upload: build  ## upload built targets to the Raspberry Pi
	ssh $(UPKIE_HOST) mkdir -p $(PROJECT_NAME)
	ssh $(UPKIE_HOST) sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after \
		--exclude .git* \
		--exclude .pixi \
		--exclude .pytest_cache \
		--exclude .ruff_cache \
		--exclude __pycache__ \
		--exclude bazel-$(CURDIR_NAME) \
		--exclude bazel-$(PROJECT_NAME)/ \
		--exclude bazel-out/ \
		--exclude bazel-testlogs/ \
		--exclude docs/ \
		--exclude logs/ \
		--exclude spines/cache/ \
		--exclude tools/bazel \
		--exclude tools/raspios/ \
		--progress $(CURDIR)/ $(UPKIE_HOST):$(PROJECT_NAME)/

# REMOTE TARGETS
# ==============

run_mock_spine:  ### run the pi3hat spine in mock mode on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine -- --mock

# NB: `make run_pi3hat_spine` appears in build instructions
run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine

run_readonly_spine:  ### run the pi3hat spine in read-only mode on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat_spine -- --readonly
