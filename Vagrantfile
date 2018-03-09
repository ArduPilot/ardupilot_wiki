# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/xenial64"

	config.vm.provision "shell", inline: <<-SHELL
                set -e
                set -x

		apt-get -y update
                apt-get install -y unzip git imagemagick mercurial python-pip

		easy_install -U pip
        
                # Install sphinx from ArduPilot repository
                pip install git+https://github.com/ArduPilot/sphinx_rtd_theme.git -UI
                pip install sphinx==1.3.6
        
                #from https://bitbucket.org/birkenfeld/sphinx-contrib/overview
                #would like to get from pip but there is no version on pypi
		hg clone https://bitbucket.org/birkenfeld/sphinx-contrib
		pushd ./sphinx-contrib/youtube/
		python setup.py build
		python setup.py install
		popd

                # and a vimeo plugin:
		hg clone https://bitbucket.org/jdouglass/sphinxcontrib.vimeo
		pushd sphinxcontrib.vimeo
		python setup.py build
		python setup.py install
		popd

	SHELL
end
