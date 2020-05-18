# makefile for building docs incrementally

ALL_WIKIS=copter plane rover antennatracker dev planner planner2 ardupilot mavproxy

ALL_CLEAN=$(addsuffix _clean,$(ALL_WIKIS))
ALL_HTML=$(addsuffix _html,$(ALL_WIKIS))

all: $(ALL_HTML)

clean: $(ALL_CLEAN)

%_common: %
	echo "Copying common files to $</source/docs"
	rsync -av common/source/docs/common-*rst $</source/docs/

%_clean: %
	$(MAKE) -C $< clean

%_html: % %_common
	$(MAKE) -C $< html

