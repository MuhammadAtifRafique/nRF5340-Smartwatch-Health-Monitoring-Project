# nRF5340 Smartwatch Health Monitoring Project

This project interfaces the nRF5340 with MPU6050, MAX30102, and BMP280 sensors to perform health monitoring as part of a smartwatch project. It calculates SPO2, Heart Rate, and Stress using MAX30102, Awake Check and Step Counting using MPU6050, and Temperature/Pressure using BMP280. The project is developed using Zephyr and nRF Connect SDK version 2.5.0.


## Introduction

This project integrates nRF5340 with MPU6050, MAX30102, and BMP280 sensors for health monitoring, specifically designed for a smartwatch application. The sensor data is printed using the RTT Console for real-time monitoring.

## Hardware Connections

1. **nRF5340 Interface With MPU6050**

   | NRF5340 Pin | MPU6050 Pin |
   |-------------|-------------|
   | VDD         | VCC         |
   | SDA(P1.02)  | SDA         |
   | SCL(P1.03)  | SCL         |
   | GND         | GND         |

2. **nRF5340 Interface With MAX30102**

   | NRF5340 Pin | MAX30102 Pin |
   |-------------|--------------|
   | VDD         | VCC          |
   | SDA(P1.02)  | SDA          |
   | SCL(P1.03)  | SCL          |
   | GND         | GND          |

3. **nRF5340 Interface With BMP280**

   | NRF5340 Pin | BMP280 Pin |
   |-------------|------------|
   | VDD         | VCC        |
   | SDA(P1.02)  | SDA        |
   | SCL(P1.03)  | SCL        |
   | GND         | GND        |

## Software Dependencies

This project relies on the following software dependencies:

- Zephyr RTOS
- nRF Connect SDK version 2.5.0

Make sure to install these dependencies before proceeding with building and flashing the project.

## Calculations

- **MAX30102:**
  - SPO2
  - Heart Rate
  - Stress

- **MPU6050:**
  - Awake Check
  - Step Counting

- **BMP280:**
  - Temperature
  - Pressure

## Building and Flashing

Follow these steps to build and flash the project:

1. Clone the repository: `git clone https://github.com/your_username/your_project.git`
2. Navigate to the project directory: `cd your_project`
3. Build the project: `west build -b nrf5340dk_nrf5340_cpuapp`
4. Flash the project: `west flash`

## Usage

I have using RTT Console for real time monitoring.

