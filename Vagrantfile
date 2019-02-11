# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/bionic64"

  config.vm.provision "shell", inline: <<-SHELL
    set -e
    set -x

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

  SHELL
end
