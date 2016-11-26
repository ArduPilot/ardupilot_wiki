# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/trusty64"

	config.vm.provision "shell", inline: <<-SHELL
		apt-get -y update

		# Basic essential toolkit
		#apt-get -y install wget
		#apt-get -y install build-essential
		#apt-get -y install python-dev
		apt-get -y install python-pip
		easy_install -U pip
        
        # install imagemagick - useful for converting files if needed
        apt-get -y install imagemagick
        
        # Install git and sphinx from git
        apt-get -y install git
        pip install git+https://github.com/hamishwillee/sphinx_rtd_theme.git -UI



		#echo "[ArduPilot]: Sphinx"
		#pip install sphinx
		#pip install sphinx_3dr_theme
		#pip install -U sphinx_3dr_theme
        #pip install -U sphinx_rtd_theme
        
		echo "[ArduPilot]: Sphinx Youtube" 
        #from https://bitbucket.org/birkenfeld/sphinx-contrib/overview
        #would like to get from pip but there is no version on pypi
		apt-get -y install mercurial
		hg clone https://bitbucket.org/birkenfeld/sphinx-contrib
		cd ./sphinx-contrib/youtube/
		python setup.py build
		python setup.py install
        

		cd /vagrant

		#echo "[ArduPilot]: Sphinx Docs "
		#cd docs/
		#make clean
		#make html
	SHELL
end
