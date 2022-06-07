# Makefile for Sphinx documentation
#
# Copyright 2022 Stéphane Caron

# You can also set these variables from the command line.
DOXYGEN      = doxygen
HTMLDIR      = html
REMOTEDIR    = tasts-robots.org:public_html/doc/upkie_locomotion

.PHONY: clean html open upload

html:
	(cd .. && $(DOXYGEN) Doxyfile)

clean:
	rm -rf $(HTMLDIR)

open:
	firefox $(HTMLDIR)/index.html

upload: clean html
	rsync -auvz --delete-after $(HTMLDIR)/ $(REMOTEDIR)/
