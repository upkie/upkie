# Makefile for upkie_image_builder
#
# SPDX-License-Identifier: Apache-2.0

CURL=curl -fsSL --compressed

all: raspios_upkie.img

clean:
	rm -rf local *.img

rebuild: clean raspios_upkie.img

local:
	mkdir local

local/%: ../% local
	cp -f $(CURDIR)/$< $@

micromamba: local
	$(CURL) "https://github.com/mamba-org/micromamba-releases/releases/download/1.5.8-0/micromamba-linux-aarch64" -o local/micromamba

pi3hat_spine: local
	$(CURL) "https://github.com/upkie/upkie/releases/latest/download/pi3hat_spine" -o local/pi3hat_spine

scripts: local/configure_cpu_isolation.py local/hard_rezero local/stop_servos local/upkie_tool

raspios_upkie.img: micromamba pi3hat_spine scripts
	docker pull mkaczanowski/packer-builder-arm:latest
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest init .
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest build .
