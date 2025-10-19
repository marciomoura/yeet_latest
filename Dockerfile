FROM debian:trixie

USER root

# Create user
ARG USER=user
ARG UID=1001
ARG GID=1001
RUN groupadd -r -g ${GID} ${USER} && \
    useradd -r -u ${UID} -g ${GID} -m -c "${USER} account" -d /home/${USER} -s /bin/bash ${USER}

RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    ca-certificates \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

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
    # Debug tools
    openocd \
    gdb-multiarch \
    # Additional utilities
    ca-certificates \
    file \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CMake
ARG CMAKE_VERSION=3.25.2
RUN wget -qO- https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz \
    | tar --strip-components=1 -xz -C /usr/local

# Install ARM toolchain
ARG ARM_TOOLCHAIN_VERSION=13.3.rel1
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz \
    | tar -xJ -C /opt
ENV PATH="/opt/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi/bin:$PATH"

# Install task
ARG TASK_VERSION=v3.45.4
RUN wget -qO- https://github.com/go-task/task/releases/download/${TASK_VERSION}/task_linux_amd64.tar.gz | tar -xz -C /usr/local/bin

ARG STM32F4XX_HAL_DRIVER_VERSION=v1.8.2
ARG CMSIS_DEVICE_F4_VERSION=v2.6.9
ARG CMSIS_CORE_VERSION=5.9.0
ARG STM32_MW_USB_HOST_VERSION=v3.5.1

RUN git clone --depth 1 --branch ${STM32F4XX_HAL_DRIVER_VERSION} \
    https://github.com/STMicroelectronics/stm32f4xx_hal_driver.git \
    /opt/STM32/stm32f4xx_hal_driver

RUN git clone --depth 1 --branch ${CMSIS_DEVICE_F4_VERSION} \
    https://github.com/STMicroelectronics/cmsis_device_f4.git \
    /opt/STM32/cmsis_device_f4

RUN git clone --depth 1 --branch ${CMSIS_CORE_VERSION} \
    https://github.com/ARM-software/CMSIS_5.git \
    /opt/STM32/cmsis_5

RUN git clone --depth 1 --branch ${STM32_MW_USB_HOST_VERSION} \
    https://github.com/STMicroelectronics/stm32_mw_usb_host.git \
    /opt/STM32/stm32_mw_usb_host

# ENV STM32_HAL_DIR=/opt/STM32/stm32f4xx_hal_driver
# ENV CMSIS_DEVICE_DIR=/opt/STM32/cmsis_device_f4
# ENV CMSIS_CORE_DIR=/opt/STM32/cmsis_5/CMSIS/Core
# ENV STM32_USB_HOST_DIR=/opt/STM32/stm32_mw_usb_host

# Install base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ripgrep \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

USER ${USER}
WORKDIR /home/${USER}

RUN mkdir -p ~/.local/bin
ENV PATH="/home/${USER}/.local/bin:${PATH}"

# Install uv
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

RUN pip install --no-cache --break-system-packages \
    cmakelang

# Set up environment
ENV CCACHE_DIR=/home/${USER}/.ccache
ENV UV_CACHE_DIR=/home/${USER}/.uv
ENV UV_LINK_MODE=copy
ENV VIRTUAL_ENV=/home/${USER}/.venv

RUN mkdir -p ${CCACHE_DIR} ${UV_CACHE_DIR} ${VIRTUAL_ENV}
