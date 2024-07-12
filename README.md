# STM32F407 Signal Generation and FFT Project

This project demonstrates signal generation using DAC on STM32F407 microcontroller, capturing the signal using ADC, and performing FFT analysis on the acquired data.

## Table of Contents
- [Overview](#overview)
- [Components Used](#components-used)
- [Setup Instructions](#setup-instructions)
  - [Prerequisites](#prerequisites)
  - [Setting Up the Environment](#setting-up-the-environment)
  - [Configuring the Project](#configuring-the-project)
- [Signal Types](#signal-types)
- [FFT Configuration](#fft-configuration)
- [Running the Project](#running-the-project)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project utilizes the STM32F407 microcontroller to generate various types of signals (sinewave, triangle wave, square wave) using the DAC module. The generated signals are then read back using the ADC module. The captured data is processed using FFT (Fast Fourier Transform) to analyze frequency components.

## Components Used

- STM32F407 microcontroller
- ARM CMSIS Library for DSP (Digital Signal Processing)
- FreeRTOS for task management
- STM32CubeMX for initial project setup
- EWARM IAR development and debugging

## Setup Instructions

### Prerequisites

- Install STM32CubeMX and EWARM IAR to compile and flash the firmware onto the STM32F407 microcontroller.
- Basic knowledge of C programming and embedded systems development.

### Setting Up the Environment
