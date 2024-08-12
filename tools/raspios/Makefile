# Makefile for upkie_image_builder
#
# SPDX-License-Identifier: Apache-2.0

CURL=curl -fsSL --compressed

all: raspios_upkie.img

clean:
	rm -f provision/configure_cpu_isolation.py
	rm -f provision/hard_rezero
	rm -f provision/micromamba
	rm -f provision/pi3hat_spine
	rm -f provision/stop_servos
	rm -f provision/upkie_tool
	rm -f raspios_upkie.img

rebuild: clean raspios_upkie.img

provision/%: ../%
	cp -f $(CURDIR)/$< $@

provision/%: ../setup/%
	cp -f $(CURDIR)/$< $@

micromamba:
	$(CURL) "https://github.com/mamba-org/micromamba-releases/releases/download/1.5.8-0/micromamba-linux-aarch64" -o provision/micromamba

pi3hat_spine:
	$(CURL) "https://github.com/upkie/upkie/releases/latest/download/pi3hat_spine" -o provision/pi3hat_spine

scripts: provision/configure_cpu_isolation.py provision/cpufreq_ondemand provision/cpufreq_performance provision/hard_rezero provision/stop_servos provision/upkie_tool provision/vcgenall

raspios_upkie.img: micromamba pi3hat_spine scripts
	docker pull mkaczanowski/packer-builder-arm:latest
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest init .
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest build .
