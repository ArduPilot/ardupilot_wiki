#!/bin/bash

export PYTHONUNBUFFERED=1

(
echo "$(date) starting"
git fetch
git reset --hard HEAD
python update.py --clean True

echo "$(date) done"
) >> build.log 2>&1

#       echo "Failed planner common_pages for $dst" | mail -s "common pages $dst" andrew-3dr@tridgell.net,hamish@3drobotics.com

