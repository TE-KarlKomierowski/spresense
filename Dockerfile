FROM phusion/baseimage:0.11

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

RUN apt-get update
RUN apt-get install -y wget clang-format 
RUN wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
RUN mkdir -p /spresenseenv/usr
RUN bash -c 'echo "#!/bin/bash" > /usr/bin/sudo'
RUN bash -c 'echo "\$@" >> /usr/bin/sudo' 
RUN chmod +x /usr/bin/sudo
RUN echo $HOME
ENV HOME /
RUN echo $HOME
RUN bash  ./install-tools.sh
ENV PATH "$PATH:/spresenseenv/usr/bin"
RUN ldconfig

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
