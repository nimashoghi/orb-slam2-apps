ARG BASE
ARG SLAM_VERSION=latest
FROM nimashoghi/${BASE}-orb-slam2:${SLAM_VERSION}

# Install fmtlib
WORKDIR /build/fmt
RUN wget https://github.com/fmtlib/fmt/releases/download/5.3.0/fmt-5.3.0.zip \
    && unzip fmt-5.3.0.zip \
    && cd fmt-5.3.0 \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && make \
    && make install

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
