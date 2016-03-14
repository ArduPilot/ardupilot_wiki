# -*- coding: utf-8 -*-
#
# This contains common configuration information for the ardupilot wikis. 
# This information is imported by the conf.py files in each of the sub wikis
#

#Set False to re-enable warnings for non-local images.
disable_non_local_image_warnings=True


#wiki_base_url='https://dl.dropboxusercontent.com/u/3067678/share2/wiki'
#intersphinx_base_url=wiki_base_url+'/%s/build/html/'

wiki_base_url='http://new.ardupilot.org/wiki/'
intersphinx_base_url=wiki_base_url+'/%s/'


# Where to point the base of the build for the main site menu
html_context= {'target':wiki_base_url}
# This needs to change to the actual URL root once the theme updated.

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'copter': (intersphinx_base_url % 'copter',
                                  None),
                       'plane': (intersphinx_base_url % 'plane',
                                  None),
                       'rover': (intersphinx_base_url % 'rover',
                                  None),
                       'planner': (intersphinx_base_url % 'planner',
                                  None),
                       'planner2': (intersphinx_base_url % 'planner2',
                                  None),
                       'dev': (intersphinx_base_url % 'dev',
                                  None),
                       'antennatracker': (intersphinx_base_url % 'antennatracker',
                                  None),
                       'ardupilot': (intersphinx_base_url % 'ardupilot',
                                  None),
                                  }