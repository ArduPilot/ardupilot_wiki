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

mkdir -p /var/sites/wiki-backup
chown -R vagrant.vagrant /var/sites/wiki-backup

cat <<"EOF" | sudo -H -u vagrant bash

mkdir -p build_wiki

pushd build_wiki
mkdir -p logs

# trees to partially clone
for i in ardupilot_wiki sphinx_rtd_theme; do
  if [ -e $i ]; then
    pushd $i
    git pull
    popd
  else
    git clone https://github.com/ardupilot/$i --depth 1
  fi
done

# trees to fully clone
for i in ardupilot; do
  if [ -e $i ]; then
    pushd $i
    git pull
    popd
  else
    git clone https://github.com/ardupilot/$i
  fi
done

EOF

echo "---------- $0 end ----------"
