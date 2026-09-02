# SPDX-License-Identifier: Apache-2.0
#
# Makefile for Sphinx documentation

# You can also set these variables from the command line.
DOXYGEN = doxygen
OUTPUTDIR = html

# Figures generated from docs/figures/, as opposed to images committed to
# docs/images/ (photographs from the build instructions, screenshots, ...)
GENERATED_FIGURES = images/action-observation-loop images/floor-contact images/observers images/state-machine images/wheel-odometry

.PHONY: html
html: figures  ## build the documentation
	(cd .. && $(DOXYGEN) $(CURDIR)/Doxyfile)

.PHONY: figures
figures: $(addsuffix .eps,$(GENERATED_FIGURES)) $(addsuffix .png,$(GENERATED_FIGURES))

images/%.eps : figures/%.dot
	dot $< -Teps -o $@

images/%.png : figures/%.dot
	dot $< -Tpng -o $@

images/%.svg : figures/%.dot
	dot $< -Tsvg -o $@

images/%.eps : figures/%.svg
	inkscape --without-gui --file=$< --export-eps=$@

images/%.png : figures/%.svg
	inkscape --without-gui --export-area-page --file=$< --export-png=$@

# Help snippet from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-24s\033[0m %s\n", $$1, $$2}'

.PHONY: clean
clean:  ## remove all generated files
	rm -rf $(OUTPUTDIR)
	rm -f $(addsuffix .eps,$(GENERATED_FIGURES)) $(addsuffix .png,$(GENERATED_FIGURES))

.PHONY: open
open:  ## open the locally built documentation in the default Web browser
	xdg-open $(OUTPUTDIR)/index.html
