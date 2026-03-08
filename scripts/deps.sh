#!/bin/bash

set -euxo pipefail

sudo apt update
sudo apt install -y \
	cmake \
	build-essential \
	git \

