#!/bin/bash
    apt-get -y update
    apt-get install -y unzip git imagemagick mercurial python-pip curl
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python get-pip.py && rm -f get-pip.py
    # Install sphinx
    pip install -U sphinx==1.8.3
    # Install sphinx theme from ArduPilot repository
    pip install git+https://github.com/ArduPilot/sphinx_rtd_theme.git -UI
    # and a youtube plugin:
    pip install git+https://github.com/sphinx-contrib/youtube.git -UI
    # and a vimeo plugin:
    hg clone https://bitbucket.org/jdouglass/sphinxcontrib.vimeo
    pushd sphinxcontrib.vimeo
    python setup.py build
    python setup.py install
    popd
