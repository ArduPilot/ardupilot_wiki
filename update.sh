#!/bin/bash
# check for changes in docs and run sphinx

set -e
set -x

export PYTHONUNBUFFERED=1

cd $HOME/build_wiki

START=$(date +%s)

############################
# grab a lock file. Not atomic, but close :)
# tries to cope with NFS
lock_file() {
        lck="$1"
        pid=`cat "$lck" 2> /dev/null`

        if test -f "$lck" && kill -0 $pid 2> /dev/null; then
	    LOCKAGE=$(($(date +%s) - $(stat -c '%Y' "build.lck")))
	    test $LOCKAGE -gt 30000 && {
                echo "old lock file $lck is valid for $pid with age $LOCKAGE seconds"
	    }
            return 1
        fi
        /bin/rm -f "$lck"
        echo "$$" > "$lck"
        return 0
}


lock_file build.lck || {
    echo "$(date +%s) already locked" >>build.lck.log
    exit 1
}

progress() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] update.sh: $*"
}

LOG_TIMESTAMP="$(date '+%Y-%m-%d-%H:%M:%S')"
LOGFILE="logs/update-$LOG_TIMESTAMP.log"
progress "update.sh starting (see $LOGFILE)"

test -n "$FORCEBUILD" || {
    progress "Fetching ardupilot_wiki"
    (cd ardupilot_wiki && git fetch)
    progress "Fetching sphinx_rtd_theme"
    (cd sphinx_rtd_theme && git fetch)

    changed=0
    progress "Getting oldhash for ardupilot_wiki"
    oldhash=$(cd ardupilot_wiki && git rev-parse origin/master)
    progress "Getting newhash for ardupilot_wiki"
    newhash=$(cd ardupilot_wiki && git rev-parse HEAD)
    [ "$oldhash" = "$newhash" ] || {
        progress "ardupilot_wiki has changed $newhash $oldhash"
        changed=1
    }
    
    progress "Getting oldhash for sphinx_rtd_theme"
    oldhash=$(cd sphinx_rtd_theme && git rev-parse origin/master)
    progress "Getting newhash for sphinx_rtd_theme"
    newhash=$(cd sphinx_rtd_theme && git rev-parse HEAD)
    [ "$oldhash" = "$newhash" ] || {
        progress "sphinx_rtd_theme has changed $newhash $oldhash"
        changed=1
    }

    progress "Fetching parameters"
    PARAMSITES="ArduPlane ArduCopter AntennaTracker Rover AP_Periph Blimp"
    mkdir -p old_params new_params
    for site in $PARAMSITES; do
        wget "https://autotest.ardupilot.org/Parameters/$site/Parameters.rst" -O new_params/$site.rst
    done

    progress "Comparing parameters"
    for site in $PARAMSITES; do
        if ! cmp new_params/$site.rst old_params/$site.rst; then
            progress "$site.rst has changed"
            cp new_params/$site.rst old_params/$site.rst
            changed=1
        fi
    done
    
    LOGMESSAGESITES="Plane Copter Tracker Rover Blimp"
    mkdir -p old_logmessages new_logmessages
    for site in $LOGMESSAGESITES; do
        wget "https://autotest.ardupilot.org/LogMessages/$site/LogMessages.rst" -O new_logmessages/$site.rst
    done

    for site in $LOGMESSAGESITES; do
        if ! cmp new_logmessages/$site.rst old_logmessages/$site.rst; then
            progress "$site.rst has changed"
            cp new_logmessages/$site.rst old_logmessages/$site.rst
            changed=1
        fi
    done

    [ $changed = 1 ] || {
	progress "Nothing changed; no rebuild required, exiting"
	exit 0
    }
}

progress "update.sh starting build"

(
date

progress "Updating ardupilot_wiki"
pushd ardupilot_wiki
git checkout -f master
git fetch origin
git submodule update
git reset --hard origin/master
git clean -f -f -x -d -d
popd

progress "Updating sphinx_rtd_theme"
pushd sphinx_rtd_theme
git checkout -f master
git fetch origin
git submodule update
git reset --hard origin/master
git clean -f -f -x -d -d
python3 -m pip install --user -U .
popd

cd ardupilot_wiki
find -name "parameters*rst" -delete # Clean possible built and cached parameters files

END_UPDATES=$(date +%s)

progress "Starting to build multiple parameters pages"
python3 build_parameters.py || {
    progress "build_parameters.py failed"
    exit 1
}
END_BUILD_MPARAMS=$(date +%s)
MPARAMS_TIME=$(echo "($END_BUILD_MPARAMS - $END_UPDATES)" | bc)
progress "Time to run build_parameters.py: $MPARAMS_TIME seconds"

progress "Starting to build the wiki"
# python3 update.py --clean --parallel 4 # Build without versioning for parameters. It is better for editing wiki.
python3 update.py --destdir /var/sites/wiki/web --clean --paramversioning --parallel 1 --enablebackups --verbose || {
    progress "update.py failed"
    exit 1
}

END_BUILD_WIKI=$(date +%s)
WIKI_TIME=$(echo "($END_BUILD_WIKI - $END_BUILD_MPARAMS)/60" | bc)
progress "Time to build the wiki itself: $WIKI_TIME minutes"
SCRIPT_TIME=$(echo "($END_BUILD_WIKI - $START)/60" | bc)
progress "Time to run the full script: $SCRIPT_TIME minutes"


) 2>&1 | tee logs/update-latest.log >$LOGFILE || {
    progress "update.sh failed; see $LOGFILE"
}

cat $LOGFILE >> logs/update.log

progress "update.sh finished"
