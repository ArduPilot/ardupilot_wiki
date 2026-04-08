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

  # default to focal for building the Wiki:
  config.vm.box = "ubuntu/focal"

  # https://releases.ubuntu.com
  # https://ubuntu.com/about/release-cycle
  # https://en.wikipedia.org/wiki/Ubuntu_version_history#Table_of_versions

  # 20.04 LTS Standard Support EOL May 2025
  config.vm.define "focal", autostart: true do |focal|
    focal.vm.box = "ubuntu/focal64"
    focal.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (focal)"
    end
    focal.vm.provision "shell", path: "./scripts/initvagrant.sh"
    focal.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "16384"]
    end
  end
end
