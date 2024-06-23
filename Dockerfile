# Use an official Ubuntu base image
FROM ubuntu:20.04

# Set environment variables to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    libopencv-dev \
    libeigen3-dev

# Create a working directory
WORKDIR /usr/src/app

# Copy the current directory contents into the container at /usr/src/app
COPY . .

