FROM phusion/baseimage:0.11

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

RUN apt-get update
RUN apt-get install -y wget clang-format
RUN ln -s /usr/bin/sha256sum /usr/bin/shasum
RUN wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
RUN bash install-tools.sh
RUN ldconfig

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
