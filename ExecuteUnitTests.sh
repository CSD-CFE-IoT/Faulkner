#!/bin/bash
# This script executes all unit tests for Faulkner
# NOTE: use Python ver 2.7

pushd test
python -m unittest discover
popd