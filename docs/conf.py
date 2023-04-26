# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys
import sphinx_bernard_theme
from typing import List, Final

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Experimental Robotics Project 2022"
copyright = "2023, Omotoye Shamsudeen Adekoya"
author = "Omotoye Shamsudeen Adekoya"
release = "1.0.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

_package_type: Final[List[str]] = ["control", "knowledge", "logic", "navigation"]
for pkg_name in _package_type:
    sys.path.insert(0, os.path.abspath(f"../exprob_{pkg_name}/"))

# subprocess.call('doxygen Doxyfile.in', shell=True)
# -- Project information -----------------------------------------------------

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx.ext.coverage",
    "sphinx.ext.mathjax",
    "sphinx.ext.ifconfig",
    "sphinx.ext.viewcode",
    "sphinx.ext.githubpages",
    "sphinx.ext.napoleon",
    "sphinx.ext.inheritance_diagram",
    "breathe",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

highlight_language = "c++"
source_suffix = ".rst"
master_doc = "index"
# html_theme = 'sphinx_rtd_theme'
# html_theme = "sphinx_bernard_theme"
# html_theme_path = [sphinx_bernard_theme.get_html_theme_path()]
html_logo = "../img/genoa_logo.png"
html_theme = "furo"
html_theme_options = {
    "light_css_variables": {
        # "color-brand-primary": "red",
        # "color-brand-content": "#CC3333",
        # "sidebar_hide_name": True,
        "top_of_page_button": "edit",
    },
        "navigation_with_keys": True,
        "source_repository": "https://github.com/pradyunsg/furo/",
        "source_branch": "main",
        "source_directory": "docs/",
}
html_static_path = ["_static"]
