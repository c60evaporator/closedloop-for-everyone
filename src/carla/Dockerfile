# syntax=docker/dockerfile:1

############################
# Set the base image
############################

# Ubuntu 20.04 (ships with Python 3.8, provides libjpeg8 required by CARLA)
FROM ubuntu:20.04

############################
# Install Python and other dependencies
############################
# CARLA_ROOT
ARG CARLA_ROOT=/workspace/carla

# Proxy settings for apt-get/pip (passed as build args)
ARG HTTP_PROXY
ARG HTTPS_PROXY
ENV http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1
# Install OS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 python3-pip python3.8-dev \
    git curl wget ca-certificates \
    libglib2.0-0 \
    libpng16-16 libjpeg8 libtiff5 \
    && ln -sf /usr/bin/python3.8 /usr/bin/python \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
WORKDIR /workspace
COPY requirements.txt /workspace/requirements.txt
RUN pip install -r requirements.txt

# CARLA Python API: .egg ファイルを sys.path に追加
ENV CARLA_ROOT=${CARLA_ROOT}
ENV PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI/carla:${PYTHONPATH}"

# アプリケーションコードをコピー
COPY src/ /workspace/src/

############################
# Create non-root user
############################
ARG USERNAME=developer
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && chown -R ${UID}:${GID} /workspace
USER ${USERNAME}
