# Self_BalancingRobot
This repository contains a bare-metal self-balancing robot firmware for the STM32F407 Discovery board, implemented entirely at the register level (no HAL). It uses an MPU-6050 IMU (I²C) to read 3-axis acceleration and gyro data, applies a complementary filter (100 Hz loop) to estimate pitch, and drives two DC motors via TIM3 PWM channels. A simple P-controller (configurable PID) maintains balance, with deadband and power limits to handle friction and safety. The code also supports on-the-fly calibration of sensor offsets, axis remapping for different chassis orientations, and a push-button start (EXTI on PA0) to enable the control loop. UART3 (PB10/PB11) at 9600 baud provides real-time telemetry (pitch, error, output) for tuning and debugging. Feel free to explore, tweak the PID gains or filter constants, and adapt it to your own balancing platform!

# PID Tuning Note
To achieve stable balancing with this firmware, you need to adjust the PID controller gains according to your specific robot’s dynamics:

Start by setting the integral (KI) and derivative (KD) gains to zero.

Gradually increase the proportional gain (KP) until the robot starts to respond but avoid excessive oscillations.

Then slowly add a small amount of derivative gain (KD) to reduce oscillations and improve stability.

Finally, introduce the integral gain (KI) carefully to correct any steady-state error.

Each robot is different due to variations in weight, motor response, and friction, so tuning these parameters experimentally is essential for optimal performance.

## PID Tuning Demonstrations

Here are three videos demonstrating the robot’s behavior with different PID settings:

1. **Only KP active:** The robot tries to balance but may oscillate.  
 [Only KP active](./kp_only.gif)

2. **KP + KD active:** Oscillations reduce, more stable balancing.  
 [KP + KD active](./kp_kd.gif)

3. **KP + KD + KI active:** Steady-state error is corrected, achieving stable balance.  
 [KP + KD + KI active](./kp_kd_ki.gif)


# STM32F4 Balancing Robot Wiring Guide

## Component Overview
DRV8833 Pins    →    Connection
AIN1           →    PB0 (TIM3_CH1)
AIN2           →    PB5 (TIM3_CH2)
BIN1           →    PB4 (TIM3_CH3)
BIN2           →    PB1 (TIM3_CH4)
```
## Complete System Wiring Diagram

```
                    STM32F4 Board
                    ┌─────────────┐
                    │             │
         ┌──────────┤ PB6 (SCL)   │
         │     ┌────┤ PB7 (SDA)   │
         │     │    │             │
         │     │    │ PB0─────────┼─────► Motor Driver 1 IN1
         │     │    │ PB5─────────┼─────► Motor Driver 1 IN2
         │     │    │ PB4─────────┼─────► Motor Driver 2 IN1
         │     │    │ PB1─────────┼─────► Motor Driver 2 IN2
         │     │    │             │               ┌─────────┐  
         │     |    │ PB10────────┼─────►      RX |  HC-06  |
         │     |    │ PB11────────┼─────►      TX |Bluetooth|         
         │     |    │             │               | module  |
         │     |    │             |               └─────────┘
         │     |    │             |
         │     |    └─────────────┘
         │     |
         │     |
         │     |____┌─────────┐
         └──────────┤ SCL SDA │
                    │   MPU   │
                    │  6050   │
                    └─────────┘
```

## Safety Notes

1. **Power Isolation**: Use separate power supplies for logic (3.3V) and motors (6-12V)
2. **Current Protection**: Add fuses on motor power lines
3. **Decoupling Capacitors**: Add 100nF ceramic capacitors near each IC
4. **Pull-up Resistors**: Essential for I2C communication (4.7kΩ on SCL/SDA)
5. **Motor Back-EMF**: Motor drivers should have built-in protection diodes

## Troubleshooting Tips

- **MPU6050 not detected**: Check I2C pull-up resistors and connections
- **Motors not responding**: Verify PWM signals with oscilloscope
- **System instability**: Check power supply stability and grounding
- **Communication issues**: Verify USART3 baud rate and connections

## Recommended Components

- **Motor Driver**: L298N or DRV8833
- **Motors**: DC geared motors (6V-12V, 100-500 RPM)
- **Battery**: 2S LiPo (7.4V) or 6x AA batteries
- **IMU**: MPU6050 breakout board
- **Frame**: Lightweight chassis (3D printed or acrylic)

