# -*- mode: ruby -*-
# vi: set ft=ruby :

# vagrant up jammy
# vagrant ssh jammy

# # on machine:
# cd build_wiki/ardupilot_wiki/
# ./update.sh
# rsync -aPH --delete /var/sites/wiki/web/ /vagrant/web

# on desktop:
# firefox web/copter/index.html
# rm -rf web

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  # default to Ubuntu jammy for building the Wiki:
  config.vm.box = "ubuntu/jammy"

  # https://releases.ubuntu.com
  # https://ubuntu.com/about/release-cycle -- Try here first and then fall back to:
  # https://en.wikipedia.org/wiki/Ubuntu_version_history#Table_of_versions

  # 22.04 LTS Standard Support EOL May 2027
  config.vm.define "jammy", autostart: true do |jammy|
    jammy.vm.box = "ubuntu/jammy64"
    jammy.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (jammy)"
    end
    jammy.vm.provision "shell", path: "./scripts/initvagrant.sh"
    jammy.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "20480"]
    end
  end

  # 24.04 LTS Standard Support EOL May 2029
  config.vm.define "noble", autostart: false do |noble|
    noble.vm.box = "ubuntu/noble64"
    noble.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (noble)"
    end
    noble.vm.provision "shell", path: "./scripts/initvagrant.sh"
    noble.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "20480"]
    end
  end

  # 26.04 LTS Standard Support EOL May 2031
  config.vm.define "resolute", autostart: false do |resolute|
    resolute.vm.box = "ubuntu/resolute64"
    resolute.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (resolute)"
    end
    resolute.vm.provision "shell", path: "./scripts/initvagrant.sh"
    resolute.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "20480"]
    end
  end
end
