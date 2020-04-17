2. Python Coding Guidelines {#coding_guidelines_1}
============================

## I. Introduction

These are the internal **Python** guidelines for the
[machines-in-motion](https://wp.nyu.edu/machinesinmotion/) group. The same
guidelines are used in the
[Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/)

The following rules present basic guidelines for our **Python** code.

These guidelines may evolve in time so it is first
good practice to check them upon creation of a new package or code refactoring.

## II. Folder Structure and File Naming

- Only *one* **Python** package per git repository, and it must be located in
    `python/<catkin_package_name>/`.
- Executable scripts should be placed in the `scripts/` folder. And should have
    a CMake **install rule** that makes them executable upon installation.

## II. Python Coding Guidelines

- Regarding the style, follow [PEP 8](https://www.python.org/dev/peps/pep-0008/).
- Use [flake8](https://flake8.pycqa.org) and fix all issues it shows.  This will
  ensure compliance with PEP 8 and also point out some critical issues like
  usage of undefined variables.
  It is recommended to install a plugin to automatically run flake8 in your
  favourite editor.  See [List of flake8 Plugins for Popular
  Editors](flake8_plugins).
- To automatically format you code, you may use
  [black](https://black.readthedocs.io).  When doing this, add `--line-length 79`
  to not violate PEP 8.
  Note, however, that black requires Pytyon 3.6, which may not be available for
  everyone.  Therefore its usage is not mandatory.

## III. Python Version

**Use Python 3.**

In case it is not possible to avoid **Python 2** for some reason, add the
following import at the top of your files to make it easier to port the code to
**Python 3** in the future and to avoid confusion about unexpected integer
division:

    # Python 3 compatibility. It has to be the first import.
    from __future__ import print_function, division
