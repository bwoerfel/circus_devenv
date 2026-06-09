"""Sphinx configuration for ca1_motor_ctrl API documentation."""

import os
import sys

sys.path.insert(0, os.path.abspath("../.."))

# ---------------------------------------------------------------------------
# Project information
# ---------------------------------------------------------------------------

project = "ca1_motor_ctrl"
copyright = "2026, Benjamin Woerfel"
author = "Benjamin Woerfel"
release = "0.1.0"

# ---------------------------------------------------------------------------
# Extensions
# ---------------------------------------------------------------------------

extensions = [
    "breathe",
    "exhale",
    "sphinx_copybutton",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# ---------------------------------------------------------------------------
# HTML output – Furo theme with dark mode
# ---------------------------------------------------------------------------

html_theme = "furo"
html_static_path = ["_static"]

html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#0088cc",
        "color-brand-content": "#0088cc",
    },
    "dark_css_variables": {
        "color-brand-primary": "#33aaee",
        "color-brand-content": "#33aaee",
    },
    "source_repository": "https://github.com/bwoerfel/ca1_motor_ctrl",
    "source_branch": "main",
    "source_directory": "docs/source/",
}

html_title = f"{project} {release}"

# ---------------------------------------------------------------------------
# Primary domain
# ---------------------------------------------------------------------------

primary_domain = "cpp"
highlight_language = "default"  # avoid mis-lexing plain/ascii blocks as C++

suppress_warnings = [
    "myst.xref_missing",        # relative file links in README/ARCHITECTURE don't map to Sphinx pages
    "misc.highlighting_failure", # ascii-tree and .srv blocks can't be lexed as any language
]

# ---------------------------------------------------------------------------
# MyST parser (Markdown support)
# ---------------------------------------------------------------------------

myst_enable_extensions = [
    "colon_fence",
    "deflist",
]

# ---------------------------------------------------------------------------
# Breathe – reads Doxygen XML produced by docs/Doxyfile
# (Doxygen is run by build_docs.sh before Sphinx; OUTPUT_DIRECTORY is
#  docs/source/doxygen_xml with XML_OUTPUT=. so index.xml lands there directly)
# ---------------------------------------------------------------------------

breathe_projects = {
    "ca1_motor_ctrl": "./doxygen_xml",
}
breathe_default_project = "ca1_motor_ctrl"
breathe_default_members = ("members", "undoc-members")

# ---------------------------------------------------------------------------
# Exhale – auto-generates API RST files from the Breathe XML
# Doxygen is NOT run by Exhale; build_docs.sh handles it explicitly.
# ---------------------------------------------------------------------------

exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "API Reference",
    "doxygenStripFromPath": "../..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": False,
}
