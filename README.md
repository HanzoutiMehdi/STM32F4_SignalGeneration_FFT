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

## Configuring the Project
Ensure the following peripherals are correctly configured in STM32CubeMX:

TIM3 for triggering ADC conversions.
TIM6 for triggering DAC updates (depending on the signal type).
ADC1 with appropriate settings for continuous conversion mode and DMA.
DAC with configurations specific to the chosen signal type.

## Signal Types
This project supports various signal types:

SIN_WAVE: Generates a sine wave using pre-defined values stored in the sinewave[] array.
TRIANGLE_WAVE: Generates a triangle wave using the DAC's triangle wave generation feature.
SQUARE_WAVE: Generates a square wave using GPIO toggling.

## FFT Configuration
FFT is configured as follows:

FFT size (FFT_SIZE) is set to 1024 for analyzing frequency components.
ARM CMSIS DSP library (arm_cfft_radix4_f32 and arm_cmplx_mag_f32) is used for FFT processing.
Running the Project
Narrator: To run the project:

## Compile the project using EWARM IAR.
Flash the compiled firmware onto the STM32F407 microcontroller.
Monitor the LED (GPIOD) toggling to indicate FFT processing.
Use a serial monitor or debugger to observe FFT results and verify signal generation.
