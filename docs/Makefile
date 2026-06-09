SPHINXOPTS    ?= -W --keep-going
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

.PHONY: help html clean livehtml

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

html:
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@echo "Build finished. Open build/html/index.html"

clean:
	@rm -rf "$(BUILDDIR)" "$(SOURCEDIR)/api" "$(SOURCEDIR)/doxygen_xml"
	@echo "Build directory cleaned."

doxygen:
	doxygen Doxyfile

SPHINXAUTOBUILD ?= sphinx-autobuild

livehtml:
	$(SPHINXAUTOBUILD) "$(SOURCEDIR)" "$(BUILDDIR)/html" $(SPHINXOPTS) $(O)
