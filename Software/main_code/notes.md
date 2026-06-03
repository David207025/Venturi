# Venturi

## Sensor PCB

- ADS7961 8-bit ADC for high-speed sensor data acquisition
- 16x QRE113 sensors for line following
- SPI communication with main PCB for data transfer
- continuous ADC sampling with 20MHz SPI clock
- DMA for efficient data transfer to ESP32

## Main PCB

- ICM-20648 IMU
- 2x DRV8212 motor drivers
- ESP32-P4-WIFI6 dev board
- XT60 connectors for battery
- 2x 6-pin SMD header footprint for motor connections

## Code

- Arduino Framework
- FreeRTOS for task management
- SPI communication between ESP32, IMU and sensor PCB
- DMA for parallel SPI communication
- PID control loop for motor speed control
    - Possibly AI inference for adaptive control
    - History-based inputs (last 20 values)
    - 20 input neurons, 2 layers with 10 neurons each, 2 output neurons for motor control signals
- Sensor data processing and fusion for accurate state estimation
- Real-time telemetry and debugging output over UDP
- OTA updates for firmware maintenance and improvements
- Power management features to optimize battery life

### Code Structure

- Core 0:
    - Sensor data acquisition and processing
    - PID control loop and motor control
    - AI inference for adaptive control
    - SPI read start for IMU and sensor PCB
    - DMA handling for SPI communication
- Core 1:
    - Telemetry and debugging output over UDP
    - Real-time debugging and logging
    - OTA update handling

#### Timeline

Core 0:

1. Core 0 starts an SPI read for the sensor PCB, using DMA for efficient data transfer.
2. Once DMA interrupt triggers, the core0 loop processes the 16 channels into a steering value using weighted average.
3. Core 0 then swaps the pointer for the next DMA transfer
4. Core 0 sends another DMA SPI read for the ADC
5. It also sends a trigger to Core 1
6. During the processing of the next ADC read, core0 uses the final steering value for PD-control or AI inference
7. Core 0 sends a notification to Core 1 to send telemetry data over UDP
8. Core 0 then sends the motor control signals to the motor drivers
9. Once it's done, it waits for the task notification from the DMA interrupt

Core 1:

1. Core 1 waits for the signal from Core 0
2. Once it receives the signal, it sends the telemetry data over UDP
3. It also checks for OTA update requests and handles them if necessary

## ESP32-S3 SPI-DMA Implementation

SPI pins:

| ESP32-P4 | ESP32-S3 | Function |
|----------|----------|----------|
| GPIO2    | GPIO13   | ADC SPI CS |
| GPIO3    | GPIO14   | ADC SPI CLK |
| GPIO4    | GPIO7    | ADC SPI MOSI |
| GPIO5    | GPIO11   | ADC SPI MISO |
| GPIO23   | GPIO2    | IMU SPI CS |
| GPIO22   | GPIO4    | IMU SPI CLK |
| GPIO21   | GPIO1    | IMU SPI MOSI |
| GPIO20   | GPIO3    | IMU SPI MISO |
| GPIO51 | GPIO6 | IMU INT |