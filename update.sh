#!/bin/bash
# check for changes in docs and run sphinx

export PYTHONUNBUFFERED=1

cd $HOME/build_wiki || exit 1

START=$(date +%s)

test -n "$FORCEBUILD" || {
    (cd ardupilot_wiki && git fetch > /dev/null 2>&1)
    (cd sphinx_rtd_theme && git fetch > /dev/null 2>&1)

    changed=0
    oldhash=$(cd ardupilot_wiki && git rev-parse origin/master)
    newhash=$(cd ardupilot_wiki && git rev-parse HEAD)
    [ "$oldhash" = "$newhash" ] || {
        echo "ardupilot_wiki has changed $newhash $oldhash"
        changed=1
    }
    
    oldhash=$(cd sphinx_rtd_theme && git rev-parse origin/master)
    newhash=$(cd sphinx_rtd_theme && git rev-parse HEAD)
    [ "$oldhash" = "$newhash" ] || {
        echo "sphinx_rtd_theme has changed $newhash $oldhash"
        changed=1
    }

    PARAMSITES="ArduPlane ArduCopter AntennaTracker Rover"
    mkdir -p old_params new_params
    for site in $PARAMSITES; do
        wget "https://autotest.ardupilot.org/Parameters/$site/Parameters.rst" -O new_params/$site.rst 2> /dev/null
    done

    for site in $PARAMSITES; do
        if ! cmp new_params/$site.rst old_params/$site.rst; then
            echo "$site.rst has changed"
            cp new_params/$site.rst old_params/$site.rst
            changed=1
        fi
    done
    
    LOGMESSAGESITES="Plane Copter Tracker Rover"
    mkdir -p old_logmessages new_logmessages
    for site in $LOGMESSAGESITES; do
        wget "https://autotest.ardupilot.org/LogMessages/$site/LogMessages.rst" -O new_logmessages/$site.rst 2> /dev/null
    done

    for site in $LOGMESSAGESITES; do
        if ! cmp new_logmessages/$site.rst old_logmessages/$site.rst; then
            echo "$site.rst has changed"
            cp new_logmessages/$site.rst old_logmessages/$site.rst
            changed=1
        fi
    done

    [ $changed = 1 ] || exit 0
}

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
    echo "already locked"
    exit 1
}


(
date

report() {
    cat <<EOF | mail -s 'wiki build failed' ardupilot.devel@gmail.com
A wiki build failed
EOF
}

echo "[Buildlog] Updating ardupilot_wiki at $(date '+%Y-%m-%d-%H-%M-%S')"
pushd ardupilot_wiki
git checkout -f master
git fetch origin
git submodule update
git reset --hard origin/master
git clean -f -f -x -d -d
popd

echo "[Buildlog] Updating sphinx_rtd_theme at $(date '+%Y-%m-%d-%H-%M-%S')"
pushd sphinx_rtd_theme
git checkout -f master
git fetch origin
git submodule update
git reset --hard origin/master
git clean -f -f -x -d -d
pip install --user -U .
popd

cd ardupilot_wiki
find -name "parameters*rst" -delete # Clean possible built and cached parameters files

echo "[Buildlog] Starting to build multiple parameters pages at $(date '+%Y-%m-%d-%H-%M-%S')"
python3 build_parameters.py
END_BUILD_MPARAMS=$(date +%s)
MPARAMS_TIME=$(echo "($END_BUILD_MPARAMS - $END_UPDATES)" | bc)
echo "[Buildlog] Time to run build_parameters.py: $MPARAMS_TIME seconds"

echo "[Buildlog] Starting to build the wiki at $(date '+%Y-%m-%d-%H-%M-%S')"
# python update.py --clean --parallel 4 # Build without versioning for parameters. It is better for editing wiki
python update.py --clean --paramversioning --parallel 4 --enablebackups # Enables parameters versioning and backups, should be used only on the wiki server


END_BUILD_WIKI=$(date +%s)
WIKI_TIME=$(echo "($END_BUILD_WIKI - $END_BUILD_MPARAMS)/60" | bc)
echo "[Buildlog] Time to build the wiki itself: $WIKI_TIME minutes"
SCRIPT_TIME=$(echo "($END_BUILD_WIKI - $START)/60" | bc)
echo "[Buildlog] Time to run the full script: $SCRIPT_TIME minutes"


) >> update.log 2>&1
