#!/bin/bash

set -e
set -x

echo "---------- $0 start ----------"

/vagrant/Sphinxsetup.sh
OUTDIR=/var/sites/wiki/web
OUTDIR_OLD=/var/sites/wiki/web/old
mkdir -p "$OUTDIR_OLD"
chown -R vagrant.vagrant "$OUTDIR"

echo "---------- $0 end ----------"
