#!/bin/sh

rm -rf logs *.log test-results .tox .eggs public
rm -rf docs/_build docs/SystemControl.* *.egg-info
find . -name "__pycache__" -exec rm -rf {} \;
find . -name "*.o" -exec rm -rf {} \;
find . -name "*.bak" -exec rm -rf {} \;

