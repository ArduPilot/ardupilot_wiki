#!/bin/bash

set -e
set -x

pushd /vagrant

scripts/initvagrant.sh

sudo -H -u vagrant scripts/install-server.sh

popd  # /vagrant
