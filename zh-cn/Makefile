# makefile for building docs incrementally

ALL_WIKIS=copter plane rover antennatracker dev planner planner2 ardupilot

ALL_CLEAN=$(addsuffix _clean,$(ALL_WIKIS))
ALL_HTML=$(addsuffix _html,$(ALL_WIKIS))

all: $(ALL_HTML)

clean: $(ALL_CLEAN)

%_clean: %
	$(MAKE) -C $< clean

%_html: %
	$(MAKE) -C $< html

