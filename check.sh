#!/bin/bash

./make.sh

if [ $? -ne 0 ]; then
    echo "Build failed."
    exit 0
fi

./run_unit_tests.sh
