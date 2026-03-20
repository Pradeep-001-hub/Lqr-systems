# Rocket Attitude Control using LQR Controller

## Introduction
This project focuses on the design and implementation of a Linear Quadratic Regulator (LQR) to manage the attitude control of a rocket. The goal is to ensure stable flight and precise orientation through optimal control strategies.

## Table of Contents
1. [Background](#background)
2. [Theory](#theory)
3. [Implementation](#implementation)
4. [Results](#results)
5. [Conclusion](#conclusion)

## Background
Rocket attitude control is essential for maintaining the desired orientation during flight. This involves calculating the necessary control inputs to adjust the rocket's pitch, yaw, and roll to achieve stable navigation.

## Theory
The LQR is a state feedback controller that minimizes a cost function which is a trade-off between the control effort and the state deviation from the desired trajectory. It is formulated as follows:

\[ J = \int_0^\infty (x^T Q x + u^T R u) dt \]

Where:
- \( x \) is the state vector,
- \( u \) is the control vector,
- \( Q \) is the state weighting matrix,
- \( R \) is the control weighting matrix.

## Implementation
To implement the LQR controller, the following steps are taken:
1. **System Modeling:** The rocket's dynamics are modeled using state-space representation.
2. **Controller Design:** The LQR is designed by selecting appropriate Q and R matrices based on the desired performance.
3. **Simulation:** Simulations are run to evaluate the performance of the controller.

## Results
Results are presented in terms of time response plots, performance metrics, and stability analysis. The effectiveness of the LQR controller in maintaining the desired attitude will be highlighted.

## Conclusion
The LQR controller provides an efficient means of achieving robust and optimal control for rocket attitude management. Future work may involve adaptive control strategies and real-time implementation on hardware.



*Date of creation: 2026-03-20*