#! /bin/bash

pylint --rcfile=.pylintrc $(git ls-files '*.py')