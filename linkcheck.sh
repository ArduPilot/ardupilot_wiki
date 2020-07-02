#!/bin/bash
# check for broken links for each of the wikis and create a report file for each ([wiki]BrokenLinks.log)
# It will check both internal and external links
# Note this is *very* slow to run - on the order of 20min per wiki
set -e -x

cd ./mavproxy
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../mavproxyBrokenLinks.log' -e d)
cd ../antennatracker
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../antennatrackerBrokenLinks.log' -e d)
cd ../ardupilot
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../ardupilotBrokenLinks.log' -e d)
cd ../copter
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../copterBrokenLinks.log' -e d)
cd ../dev
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../devBrokenLinks.log' -e d)
cd ../plane
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../planeBrokenLinks.log' -e d)
cd ../planner
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../plannerBrokenLinks.log' -e d)
cd ../planner2
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../planner2BrokenLinks.log' -e d)
cd ../rover
make linkcheck | tee >(sed -E -e '/broken|writing[[:space:]]output/!b' -e 'w ../rover2BrokenLinks.log' -e d)
