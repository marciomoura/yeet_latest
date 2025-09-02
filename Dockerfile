ARG DOCKER_IMAGE_BASE=debian:bookworm

FROM ${DOCKER_IMAGE_BASE}

USER root

ARG DEBIAN_FRONTEND=noninteractive

# Install base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tooling
    build-essential \
    clang \
    llvm \
    lld \
    ninja-build \
    make \
    pkg-config \
    # Tools
    git \
    wget \
    curl \
    python3 \
    python3-pip \
    ccache \
    gcovr \
    valgrind \
    lcov \
    clang-format \
    clang-tidy \
    gdb-multiarch \
    # Additional utilities
    ca-certificates \
    file \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Debug tools
    openocd \
    stlink-tools \
    dfu-util \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install J-Link dependencies including udev
RUN apt-get update && apt-get install -y \
    libusb-1.0-0 \
    libx11-6 \
    libx11-xcb1 \
    libxcb1 \
    libxcb-image0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxext6 \
    libxfixes3 \
    libxi6 \
    libxrandr2 \
    libxrender1 \
    libxss1 \
    libxtst6 \
    libxv1 \
    libxxf86vm1 \
    libsm6 \
    libice6 \
    libfontconfig1 \
    libfreetype6 \
    libxkbcommon0 \
    libxkbcommon-x11-0 \
    udev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a fake udevadm to satisfy J-Link's post-install script
RUN echo '#!/bin/bash\necho "Skipping udevadm in container"\nexit 0' > /usr/bin/udevadm && \
    chmod +x /usr/bin/udevadm

# Download and install J-Link software
ARG JLINK_VERSION=858
RUN wget -q --post-data 'accept_license_agreement=accepted' \
    https://www.segger.com/downloads/jlink/JLink_Linux_V${JLINK_VERSION}_x86_64.deb && \
    dpkg -i JLink_Linux_V${JLINK_VERSION}_x86_64.deb && \
    rm JLink_Linux_V${JLINK_VERSION}_x86_64.deb


ENV STM32CUBE_PATH=/opt/STM32CUBE
WORKDIR ${STM32CUBE_PATH}
ARG STM32CUBE_VERSION=v1.28.3
ARG DEVICE=F4
RUN git clone --depth 1 --branch ${STM32CUBE_VERSION} https://github.com/STMicroelectronics/STM32Cube${DEVICE}.git && \
    git -C STM32Cube${DEVICE} submodule update --init --depth 1 \
    # # These are submodules that must be initialized
    Drivers/CMSIS/Device/ST/STM32${DEVICE}xx \
    Drivers/STM32${DEVICE}xx_HAL_Driver && \
    # Clean up unnecessary stuff but keep Drivers
    rm -rf STM32Cube${DEVICE}/.git STM32Cube${DEVICE}/Projects STM32Cube${DEVICE}/_htmresc STM32Cube${DEVICE}/Utilities STM32Cube${DEVICE}/Documentation STM32Cube${DEVICE}/*.html STM32Cube${DEVICE}/*.pdf

# Install CMake
ARG CMAKE_VERSION=3.31.0
RUN wget -qO- https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz \
    | tar --strip-components=1 -xz -C /usr/local

# Install ARM toolchain
ARG ARM_TOOLCHAIN_VERSION=12.2.rel1
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz \
    | tar -xJ -C /opt
ENV PATH="/opt/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi/bin:$PATH"

# Install task
ARG TASK_VERSION=v3.44.0
RUN wget -qO- https://github.com/go-task/task/releases/download/${TASK_VERSION}/task_linux_amd64.tar.gz | tar -xz -C /usr/local

# Install STM32 tools and Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyocd \
    stm32loader

# Create user (if not already created)
ARG USER=user
ARG UID=1001
ARG GID=1001
RUN <<EOF
#!/bin/bash
if ! getent group ${GID} >/dev/null 2>&1; then
    groupadd -r -g ${GID} ${USER}
fi
if ! getent passwd ${UID} >/dev/null 2>&1; then
    useradd -r -u ${UID} -g ${GID} -m -c "${USER} account" -d /home/${USER} -s /bin/bash ${USER}
fi
EOF

USER ${USER}
WORKDIR /home/${USER}

# Set up environment
ENV CCACHE_DIR=/home/${USER}/.ccache
