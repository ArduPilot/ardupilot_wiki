#!/bin/bash

set -e
set -x

echo "---------- $0 start ----------"

DISTRIBUTION_ID=$(lsb_release -i -s)
if [ ${DISTRIBUTION_ID} == 'Ubuntu' ]; then
  DISTRIBUTION_CODENAME=$(lsb_release -c -s)
  if [ ${DISTRIBUTION_CODENAME} == 'bionic' ]; then
      # make python3 the default Python on bionic:
      sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 20
  fi
fi

sudo -H -u vagrant /vagrant/Sphinxsetup.sh
OUTDIR=/var/sites/wiki/web
OUTDIR_OLD=/var/sites/wiki/web/old
mkdir -p "$OUTDIR_OLD"
chown -R vagrant.vagrant "$OUTDIR"

echo "---------- $0 end ----------"
