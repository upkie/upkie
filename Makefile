# Makefile for upkie_locomotion targets
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
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
#
# NB: this Makefile is for Raspberry Pi targets only. Use Bazel or Bazelisk to
# build targets on your computer.

# Hostname or IP address of the Raspberry Pi
RASPI = upkie

# Project name, needs to match the one in WORKSPACE
PROJECT_NAME = upkie_locomotion

CURDATE = $(shell date --iso=seconds)
RASPUNZEL = $(CURDIR)/tools/raspunzel

.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete

# Targets
# =======

.PHONY: build
build: clean_broken_links  ## build Raspberry Pi targets
	bazel build --config=pi64 //agents/pink_balancer
	bazel build --config=pi64 //agents/test_balancer:agent
	bazel build --config=pi64 //spines:pi3hat

run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s //spines:pi3hat

run_pink_balancer:  ### run the pink balancer on the Raspberry Pi
	$(RASPUNZEL) run -v -s //agents/pink_balancer -- --config pi3hat

run_test_balancer:  ### run the test balancer on the Raspberry Pi
	$(RASPUNZEL) run -v -s //agents/test_balancer:agent -- --config pi3hat

# Upload from host to Raspberry Pi
# ================================

# Running ``raspunzel -s`` can create __pycache__ directories owned by root
# that rsync is not allowed to remove. We therefore give permissions first.
.PHONY: upload
upload: build  ## upload built targets to the Raspberry Pi
	ssh $(RASPI) sudo date -s "$(CURDATE)"
	ssh $(RASPI) mkdir -p $(PROJECT_NAME)
	ssh $(RASPI) sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded --exclude bazel-out/ --exclude bazel-testlogs/ --exclude bazel-$(PROJECT_NAME)/ --progress $(CURDIR)/ $(RASPI):$(PROJECT_NAME)/

# Help
# ====

# Help snippet from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Host:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRemote:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help
