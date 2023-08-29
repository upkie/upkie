# Makefile for Sphinx documentation
#
# Copyright 2022 Stéphane Caron

# You can also set these variables from the command line.
DOXYGEN = doxygen
OUTPUTDIR = html

images/%.eps : figures/%.svg
	inkscape --without-gui --file=$< --export-eps=$@

images/%.png : figures/%.svg
	inkscape --without-gui --export-area-page --file=$< --export-png=$@

figures: images/floor-contact.eps images/floor-contact.png images/observers.eps images/observers.png images/wheel-odometry.eps images/wheel-odometry.png

# Help snippet from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help

.PHONY: html
html: figures  ## build the documentation
	(cd .. && $(DOXYGEN) $(CURDIR)/Doxyfile)

.PHONY: clean
clean:  ## remove all generated files
	rm -rf $(OUTPUTDIR)
	rm -f $(wildcard images/*.eps) $(wildcard images/*.png)

.PHONY: open
open:  ## open the locally built documentation in Firefox
	firefox $(OUTPUTDIR)/index.html
