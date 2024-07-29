# Makefile for upkie_image_builder
#
# SPDX-License-Identifier: Apache-2.0

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Available targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-16s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

image:  ## build system image to raspios_upkie.img
	docker pull mkaczanowski/packer-builder-arm:latest
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest init .
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest build .
