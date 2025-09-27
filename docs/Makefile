# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD  ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Custom targets for LittleBot documentation

clean:
	rm -rf $(BUILDDIR)/*

html-dev:
	@$(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(BUILDDIR)/html" $(SPHINXOPTS) $(O) -E -a

serve:
	cd $(BUILDDIR)/html && python3 -m http.server 8000

install-deps:
	pip3 install sphinx sphinx-rtd-theme sphinx-autoapi breathe myst-parser

doxygen:
	@echo "Running Doxygen to generate C++ API documentation..."
	doxygen Doxyfile

docs: doxygen html
	@echo "Documentation built successfully!"
	@echo "Open $(BUILDDIR)/html/index.html in your browser"

.PHONY: clean html-dev serve install-deps doxygen docs