# Makefile for upkie_image_builder
#
# SPDX-License-Identifier: Apache-2.0

CURL=curl -fsSL --compressed
VERSION=$(shell curl -s https://api.github.com/repos/upkie/upkie/releases/latest | jq .tag_name | sed 's/^"v//; s/"$$//')
IMAGE=raspios_upkie-$(VERSION).img

build: micromamba pi3hat_spine scripts
	rm -f raspios_upkie.img
	docker pull mkaczanowski/packer-builder-arm:latest
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest init .
	docker run --rm --privileged -v /dev:/dev -v $(CURDIR):/build mkaczanowski/packer-builder-arm:latest build .
	mv raspios_upkie.img $(IMAGE)
	sha1sum $(IMAGE) $(IMAGE).sha1
	sha256sum $(IMAGE) $(IMAGE).sha256

clean:
	rm -f local/configure_cpu_isolation.py
	rm -f local/hard_rezero
	rm -f local/micromamba
	rm -f local/pi3hat_spine
	rm -f local/stop_servos
	rm -f local/upkie_tool
	rm -f *.img

rebuild: clean build

local/%: ../%
	cp -f $(CURDIR)/$< $@

local/%: ../setup/%
	cp -f $(CURDIR)/$< $@

micromamba:
	$(CURL) "https://github.com/mamba-org/micromamba-releases/releases/download/1.5.8-0/micromamba-linux-aarch64" -o local/micromamba

pi3hat_spine:
	$(CURL) "https://github.com/upkie/upkie/releases/latest/download/pi3hat_spine" -o local/pi3hat_spine

scripts: local/configure_cpu_isolation.py local/cpufreq_ondemand local/cpufreq_performance local/hard_rezero local/stop_servos local/upkie_tool local/vcgenall
