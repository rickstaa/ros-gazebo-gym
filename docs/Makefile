# Makefile for ROS Gazebo Gym sphinx documentation

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = ROS Gazebo Gym
SOURCEDIR     = source
BUILDDIR      = build
GH_PAGES_SOURCES = docs src/ros_gazebo_gym

# User-friendly check for sphinx-build
ifeq ($(shell which $(SPHINXBUILD) >/dev/null 2>&1; echo $$?), 1)
$(error The '$(SPHINXBUILD)' command was not found. Make sure you have Sphinx installed, then set the SPHINXBUILD environment variable to point to the full path of the '$(SPHINXBUILD)' executable. Alternatively you can add the directory with the executable to your PATH. If you don't have Sphinx installed, grab it from http://sphinx-doc.org/)
endif

# Put it first so that "make" without argument is like "make help".
.PHONY: help Makefile
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Create make clean command
.PHONY: clean
clean:
	@rm -rf $(BUILDDIR)/*
	@rm -rf $(SOURCEDIR)/dev/api/_autosummary
	@echo "Build folder cleaned"

# Publish gh-pages command
.PHONY: gh-pages
gh-pages:
	cd .. && \
	git submodule update --recursive && \
	git checkout gh-pages && \
	git rm -rf . && git clean -fxd && \
	git checkout main $(GH_PAGES_SOURCES) && \
	git reset HEAD && \
	cd docs && \
	make clean && \
	make html && \
	cd .. && \
	git add docs/build/html && \
	git clean -ffxd && \
	mv -fv docs/build/html/* ./ && \
	mv -fv docs/build/html/.[!.]* ./ && \
	rm -rf $(GH_PAGES_SOURCES) && \
	git add -A && \
	git commit -m "docs: Generated gh-pages for `git log main -1 --pretty=short --abbrev-commit`" && \
	git push origin gh-pages && \
	git checkout main && \
	git submodule update --recursive && \
	pnpm install && \
	pnpm run prepare && \
	git restore pnpm-lock.yaml

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	git submodule update --init --recursive
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)