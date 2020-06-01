# -*- mode: ruby -*-
# vi: set ft=ruby :


# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "ubuntu/bionic64"
  config.vm.provider "virtualbox" do |vb|
      vb.name = "ArduPilot_wiki"
  end
  config.vm.provision "shell", path: "./Sphinxsetup.sh"
  config.vm.provider "virtualbox" do |vb|
      vb.customize ["modifyvm", :id, "--memory", "2048"]
  end
end
