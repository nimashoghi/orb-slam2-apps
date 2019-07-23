FROM nimashoghi/ubuntu-xenial-orb-slam2:latest

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

RUN rm -rf /build
