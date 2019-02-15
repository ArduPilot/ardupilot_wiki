# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/bionic64"

  config.vm.provision "shell", inline: <<-SHELL
    set -e
    set -x

    apt-get -y update
    apt-get install -y unzip git imagemagick mercurial python3-pip curl

    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && rm -f get-pip.py

    # Install sphinx
    pip3 install -U sphinx==1.8.3

    # Install sphinx theme from ArduPilot repository
    pip3 install git+https://github.com/ArduPilot/sphinx_rtd_theme.git -UI

    # and a youtube plugin:
    pip3 install git+https://github.com/sphinx-contrib/youtube.git -UI

    # and a vimeo plugin:
    hg clone https://bitbucket.org/jdouglass/sphinxcontrib.vimeo
    pushd sphinxcontrib.vimeo
    python3 setup.py build
    python3 setup.py install
    popd

  SHELL
end
