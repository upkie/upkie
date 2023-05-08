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

BAZEL_DIR = bazel-bin
PROJECT_NAME = upkie_locomotion
RASPUNZEL = $(CURDIR)/tools/raspunzel
REMOTE_HOST = upkie

# Help snippet from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Host targets:"
	@grep -P '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Remote targets:"
	@grep -P '^[a-zA-Z_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

build:  ### build Raspberry Pi targets
	bazel build --config=pi64 //spines:pi3hat

run_bullet_spine:  ## run a Bullet spine with GUI
	bazel run -c opt //spines:bullet  -- --show

run_pi3hat_spine:  ## run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) -s run //spines:pi3hat

run_pink_balancer:  ## run the pink balancer on the Raspberry Pi
	$(RASPUNZEL) -s run //agents/pink_balancer -- --config pi3hat

upload: build  ## upload built targets to the Raspberry Pi
	rsync -Lrtu --delete-after $(CURDIR)/ test__/  # keep the trailing slashes
