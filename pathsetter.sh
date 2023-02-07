#!/bin/bash

echo "export PYTHONPATH=\$PYTHONPATH:"$(pwd)"/armor_py_api/scripts/armor_api/" >> ~/.bashrc
source ~/.bashrc