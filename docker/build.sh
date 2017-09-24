#!/bin/bash
set -e

THIS_DIR="$(cd "$(dirname "$0")" && pwd -P && cd - > /dev/null)"

docker build --file "$THIS_DIR/Dockerfile.cpu"             \
             --tag eurobots/carnd_capstone:latest      \
             "$THIS_DIR/.."
