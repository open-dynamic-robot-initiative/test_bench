List of flake8 Plugins for Popular Editors
==========================================

Feel free to extend the list if you find a plugin for an editor not yet listed
here or a better plugin for one that is already listed.

Note that you need to be careful to use flake8 for the correct Python version.


## Visual Studio Code

VS Code already has built-in support for flake8 but it needs to be enabled in
the configuration.  See [the official
documentation](https://code.visualstudio.com/docs/python/linting) for
instructions.


## Vim

### Syntastic

[Syntastic](https://github.com/vim-syntastic/syntastic) is a plugin for running
external style checkers that can run flake8.

    let g:syntastic_python_checkers = ['flake8']
    " to ensure flake8 checks for python 3 code:
    let g:syntastic_python_flake8_exe = 'python3 -m flake8'

### ALE

[ALE](https://github.com/dense-analysis/ale) is a plugin for running linters
asynchronously.  As it runs the linter in the background, it is preferable over
Syntastic, however, it needs Vim version 8 or later or Neovim.

To ensure using flake for Python 3, use the following configuration:

    let g:ale_python_flake8_executable = 'python3'
    let g:ale_python_flake8_options = '-m flake8'

