# -*- mode: ruby -*-
# vi: set ft=ruby :


# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  # default to focal for building the Wiki:
  config.vm.box = "ubuntu/focal"

  # 18.04 LTS EOL April 2023
  config.vm.define "bionic", autostart: false do |bionic|
    bionic.vm.box = "ubuntu/bionic64"
    bionic.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (bionic)"
    end
    bionic.vm.provision "shell", path: "./scripts/initvagrant.sh"
    bionic.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "2048"]
    end
  end

  # 20.04 LTS  EOL April 2025
  config.vm.define "focal", autostart: true do |focal|
    focal.vm.box = "ubuntu/focal64"
    focal.vm.provider "virtualbox" do |vb|
        vb.name = "ArduPilot_wiki (focal)"
    end
    focal.vm.provision "shell", path: "./scripts/initvagrant.sh"
    focal.vm.provider "virtualbox" do |vb|
        vb.customize ["modifyvm", :id, "--memory", "2048"]
    end
  end
end
