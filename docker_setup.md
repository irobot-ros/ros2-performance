# Docker Setup


## Install Docker

Remove old Docker versions

    $ sudo apt-get remove docker docker-engine docker.io

Set up the repository

    $ sudo apt-get update
    $ sudo apt-get install \
        apt-transport-https \
        ca-certificates \
        curl \
        software-properties-common

    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -


    $ sudo add-apt-repository \
       "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
       $(lsb_release -cs) \
       stable"


Install Docker CE


    $ sudo apt-get update
    $ sudo apt-get install docker-ce


Now Docker is installed on your computer.
In the following you will find some additional configuration steps which may be required depending on your particular use cases.


### Manage Docker as non-root user

**NOTE** This step is optional. If you skip it, simply add `sudo` to all the Docker commands.

Make sure that the docker group exists or create it

    $ sudo groupadd docker

Add your user to the docker group.

    $ sudo usermod -aG docker $USER

Log out and log back in so that your group membership is re-evaluated.

More information on this [here](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user)


### Allow the Docker daemon to access the network

**NOTE** This step is required if you are under the corporate network.
If you skip it and then you notice that `docker build` fails while trying to download some files, follow these steps.


Get your DNS address

    $ nmcli dev show | grep 'IP4.DNS'
    IP4.DNS[1]:                             10.0.0.2

Edit or create a file /etc/docker/daemon.json
Make sure that the "dns" field contains the first address you got from the previous command

    {
      "dns": ["10.0.0.2", "8.8.8.8"]
    }

Restart docker

     $ sudo service docker restart


More information on this issue [here](https://stackoverflow.com/questions/24151129/network-calls-fail-during-image-build-on-corporate-network/51756126#51756126).


### Allow Docker to run and build ARM containers

Using ARM containers on X86_64 laptops can be very useful, for example for cross-compilation.

This is not possible by default with Docker in Ubuntu, while it is possible with Mac OS and Windows.

There are several solutions to this problem and not all may work depending on your Ubuntu or Kernel version. You should try them one after the other until you find something working for you.

##### Test if you need this step

In order to test if you are able to use ARM containers, you can do the following:

```
docker run -it arm32v7/debian:stretch-slim
```

If it does not fail, you are fine and you can skip this section.
Otherwise try the following solutions.


##### Solution 1

```
sudo apt-get install qemu-user-static
sudo systemctl restart systemd-binfmt.service
```

##### Solution 2

```
git clone https://github.com/computermouth/qemu-static-conf.git
sudo mkdir -p /lib/binfmt.d
sudo cp qemu-static-conf/*.conf /lib/binfmt.d/
sudo systemctl restart systemd-binfmt.service
```

##### Solution 3

The last solution has to be repeated for each Dockerfile.

```
cd <Dockerfile_directory>
sudo apt-get install qemu-user-static
mkdir qemu-user-static
cp /usr/bin/qemu-*-static qemu-user-static
```

Then add a line to your Dockerfile, immediately after the `FROM` command and before any other instruction.

```
COPY qemu-arm-static /usr/bin
```

Then you can build and run your Dockerfile as usual.

More information on this issue:
 - [Running and building ARM Docker containers in x86](https://ownyourbits.com/2018/06/27/running-and-building-arm-docker-containers-in-x86/)
 - [Qemu instruction translation for ARM](https://github.com/docker/for-linux/issues/56)
 - [Debian bug report](https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=868217)
