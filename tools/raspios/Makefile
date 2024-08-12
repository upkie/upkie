# Makefile for upkie_image_builder
#
# SPDX-License-Identifier: Apache-2.0

CURL=curl -fsSL --compressed

all: raspios_upkie.img

clean:
	rm -f local/configure_cpu_isolation.py
	rm -f local/hard_rezero
	rm -f local/micromamba
	rm -f local/pi3hat_spine
	rm -f local/stop_servos
	rm -f local/upkie_tool
	rm -f raspios_upkie.img

rebuild: clean raspios_upkie.img

local/%: ../%
	cp -f $(CURDIR)/$< $@

local/%: ../setup/%
	cp -f $(CURDIR)/$< $@

micromamba:
	$(CURL) "https://github.com/mamba-org/micromamba-releases/releases/download/1.5.8-0/micromamba-linux-aarch64" -o local/micromamba

pi3hat_spine:
	$(CURL) "https://github.com/upkie/upkie/releases/latest/download/pi3hat_spine" -o local/pi3hat_spine

scripts: local/configure_cpu_isolation.py local/cpufreq_ondemand local/cpufreq_performance local/hard_rezero local/stop_servos local/upkie_tool local/vcgenall

raspios_upkie.img: micromamba pi3hat_spine scripts
	docker pull mkaczanowski/packer-builder-arm:latest
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest init .
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest build .
