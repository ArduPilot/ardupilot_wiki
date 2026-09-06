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

# A build is wedged if it is older than LOCK_MAX_AGE, or if its process tree
# has used no CPU at all for LOCK_MAX_STALL. CPU is the signal here because a
# build blocked on a dead socket accrues exactly zero while a working one
# accrues constantly. Log output is not a usable signal: Sphinx goes quiet for
# half an hour at a time while running flat out. Without this a single hung
# download stops wiki publishing indefinitely, as the lock is never reclaimed.
LOCK_MAX_AGE=30000
LOCK_MAX_STALL=1800
# "<cpu ticks> <time those ticks were first seen>"
LOCK_PROGRESS=build.lck.progress

# every descendant of $1, deepest first
descendants() {
    local child
    for child in $(pgrep -P "$1" 2>/dev/null); do
        descendants "$child"
        echo "$child"
    done
}

# CPU ticks used by $1 and its descendants, reaped children included
tree_cpu() {
    local total=0 p t
    for p in $(descendants "$1") "$1"; do
        # strip through "comm)" first: the command name can contain spaces
        t=$(sed 's/.*) //' /proc/$p/stat 2>/dev/null | awk '{print $12+$13+$14+$15}')
        test -n "$t" && total=$((total + t))
    done
    echo $total
}

# seconds since the build last used any CPU; updates the progress file
cpu_stalled_for() {
    local pid="$1" now="$2" cpu prev_cpu prev_at
    cpu=$(tree_cpu "$pid")
    # guarded rather than "< $f 2>/dev/null": the redirect is attempted before
    # the suppression applies, so a missing file still writes to stderr
    if test -r "$LOCK_PROGRESS"; then
        read -r prev_cpu prev_at < "$LOCK_PROGRESS" || true
    fi
    if test "$cpu" != "$prev_cpu" || test -z "$prev_at"; then
        echo "$cpu $now" > "$LOCK_PROGRESS"
        echo 0
        return
    fi
    echo $((now - prev_at))
}

# kill a wedged build and its children; 0 if the lock is now ours to take
break_lock() {
    local pid="$1" why="$2" victims
    # the pid may have been recycled onto something that is not ours
    case "$(ps -o args= -p "$pid" 2>/dev/null)" in
        *update.sh*) ;;
        *) echo "$(date +%s) lock pid $pid is not update.sh, left alone" >>build.lck.log
           return 1 ;;
    esac
    victims="$(descendants "$pid") $pid"
    echo "$(date +%s) breaking stale lock, pid $pid ($why), killing:" $victims >>build.lck.log
    kill -TERM $victims 2>/dev/null
    sleep 10
    kill -KILL $victims 2>/dev/null
    sleep 2
    if kill -0 "$pid" 2>/dev/null; then
        echo "$(date +%s) pid $pid survived SIGKILL, lock not taken" >>build.lck.log
        return 1
    fi
    return 0
}

lock_file() {
        lck="$1"
        pid=`cat "$lck" 2> /dev/null`

        if test -f "$lck" && kill -0 $pid 2> /dev/null; then
            now=$(date +%s)
            age=$((now - $(stat -c '%Y' "$lck")))
            stall=$(cpu_stalled_for "$pid" "$now")

            why=""
            if test $age -gt $LOCK_MAX_AGE; then why="age ${age}s"; fi
            if test $stall -gt $LOCK_MAX_STALL; then why="${why:+$why, }no CPU for ${stall}s"; fi
            if test -z "$why"; then return 1; fi
            if ! break_lock "$pid" "$why"; then return 1; fi
        fi
        /bin/rm -f "$lck" "$LOCK_PROGRESS"
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
    PARAMSITES="ArduPlane ArduCopter ArduSub AntennaTracker Rover AP_Periph Blimp"
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
    
    LOGMESSAGESITES="Plane Copter Sub Tracker Rover Blimp"
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
