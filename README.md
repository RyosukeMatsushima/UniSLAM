# uni_slam

`uni_slam` is a universal multi-agent visual SLAM system that provides real-time position and orientation data to any machine.
By sending camera images to the service, machines can receive accurate localization information. The system continuously updates and optimizes its SLAM data based on these images.
This service aims to enable all machines around the world to easily obtain accurate position information.

## Getting Started

### Prerequisites

- Docker
- Docker Compose

Ensure you have both Docker and Docker Compose installed on your system.

### Running the Application

To run the `uni_slam` application:

1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/uni_slam.git
   cd uni_slam
   ```

2. Build and run the Docker containers:
   ```bash
   docker compose up -d --build
   ```

This will build the Docker images and start the necessary containers in detached mode.

### Running Tests

To run the unit tests for `uni_slam`, you can use Docker as well:

1. Build the Docker image and start the container:
   ```bash
   docker compose up -d --build
   ```

2. Execute the project build script:
   ```bash
   docker compose exec app ./make.sh
   ```

3. Run the unit tests:
   ```bash
   docker compose exec app ./run_ctest.sh
   ```

4. Once you have finished testing, stop the Docker container:
   ```bash
   docker compose down
   ```

## Current Progress
See [progress](progress) for the current progress of the project.
