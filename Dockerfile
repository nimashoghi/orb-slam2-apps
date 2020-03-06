FROM nimashoghi/ubuntu-xenial-orb-slam2:2-13-20

# install latest c++
RUN  apt-get update && \
    apt-get install build-essential software-properties-common -y && \
    add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install gcc-snapshot -y && \
    apt-get update && \
    apt-get install gcc-6 g++-6 -y && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6 && \
    apt-get install gcc-4.8 g++-4.8 -y && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8;

# Build and install app
WORKDIR /build/app
COPY . .
RUN mkdir install \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j \
    && make install

RUN rm -rf /build/

WORKDIR /root
ENV LD_LIBRARY_PATH=/usr/local/lib
