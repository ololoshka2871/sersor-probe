FROM catthehacker/ubuntu:act-22.04

# install programs
RUN <<EOF 
apt update
apt install -y \
    curl \
    cmake \
    ninja-build \
	nodejs \
	git \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    mono-xbuild \
    mono-devel
apt clean
rm -rf /var/lib/apt/lists/*
EOF

# Install Rust nightly
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain nightly --profile minimal

# install rust targets
RUN /root/.cargo/bin/rustup target add thumbv6m-none-eabi thumbv7m-none-eabi thumbv7em-none-eabi thumbv7em-none-eabihf

ENV PATH="/root/.cargo/bin:${PATH}"
