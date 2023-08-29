# Makefile for Sphinx documentation
#
# Copyright 2022 Stéphane Caron

# You can also set these variables from the command line.
DOXYGEN      = doxygen
HTMLDIR      = html
REMOTEDIR    = tasts-robots.org:public_html/doc/upkie

images/%.eps : figures/%.svg
	inkscape --without-gui --file=$< --export-eps=$@

images/%.png : figures/%.svg
	inkscape --without-gui --export-area-page --file=$< --export-png=$@

figures: images/floor-contact.eps images/floor-contact.png images/observers.eps images/observers.png images/wheel-odometry.eps images/wheel-odometry.png
# Help snippet from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
help:
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-24s\033[0m %s\n", $$1, $$2}'
.DEFAULT_GOAL := help
.PHONY: help

html: figures  ## build the documentation
	(cd .. && $(DOXYGEN) $(CURDIR)/Doxyfile)
.PHONY: html

clean:  ## clean up
	rm -rf $(HTMLDIR)
	rm -f $(wildcard images/*.eps) $(wildcard images/*.png)
.PHONY: clean

open:  ## open the locally built documentation in Firefox
	firefox $(HTMLDIR)/index.html
.PHONY: open

upload: clean html  ## upload the docs
	rsync -auz --delete-after --progress $(HTMLDIR)/ $(REMOTEDIR)/
.PHONY: upload
