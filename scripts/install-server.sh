#!/bin/bash

set -e
set -x

# installs an approximation to the actual ArduPilot Wiki server

if [ "$UID" -eq 0 ]; then
     echo "Sorry, this script must NOT be run as ROOT!"
     exit 1
fi

INSTALL_START_DIR="$PWD"

pushd $HOME

mkdir build_wiki
pushd build_wiki
git clone https://github.com/ArduPilot/ardupilot_wiki.git
git clone https://github.com/ArduPilot/ardupilot.git
git clone https://github.com/ArduPilot/sphinx_rtd_theme.git
popd

# setup a cronjob to build the Wiki periodically.  The user's crontab
# may be altered.
mkdir cron
cp $INSTALL_START_DIR/scripts/update.cron cron/
TMP=crontab.$$
crontab -l >$TMP || true
if ! grep 'cron/update.cron' $TMP; then
    echo "*/5 * * * * $HOME/cron/update.cron" >>$TMP
    crontab $TMP
fi

for DIR in /var/sites/wiki/web/old /var/sites/wiki-backup; do
  sudo mkdir -p $DIR
  sudo chown $USER.$USER $DIR
done

# install Apache
sudo apt-get install -y apache2

# concrete
# nginx?

popd  # $HOME
