# Model Predictive Control

This project is a part of Udacity's *Self-Driving Car Nanodegree* program. The
goal of the project is to implement the controller that would invoke Model
Predictive Control method for driving the car in the simulator. 

## Model

In general, Model Predictive Control requires a dynamic model of the controlled
process to predict future states and find optimal values for control signals at
given time *t*. In case of this project, the controlled process is a car that
moves long a predefined trajectory. The car's state state consists of the
following components: 

|       |                     | 
| ----- | ------------------- |
| *x*   | x-coordinate        |
| *y*   | y-coordinate        |
| *ψ*   | orientation angle   |
| *v*   | speed               |
| *cte* | cross-track error   |
| *eψ*  | orientation error   |

The control signals (actuators) are: 

|     |                |
| --- | -------------- |
| *δ* | steering angle |
| *a* | throttle       |

The dynamic model used in this project is a kinematic motion model, defined as
follows:

*x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(ψ<sub>t</sub>) * dt*
*y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(ψ<sub>t</sub>) * dt*
*ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub>/L<sub>f</sub> * δ<sub>t</sub> * dt*
*v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt*
*cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + (v<sub>t</sub> * sin(eψ<sub>t</sub>) * dt)*
*eψ<sub>t+1</sub> = ψ<sub>t</sub> - ψdes<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub> * δ<sub>t</sub> * dt)*

## Timestep and elapsed duration 

The values for the timestep *dt* and the number of timesteps *N* have been
set to 0.1 and 15, respectively. These values mean that the optimizer will look
ahead for 1.5 seconds to calculate the optimal trajectory, with the step of 0.1
second. These values work quite well in practice with reasonable computation
time. 

## Polynomial fitting 

The waypoints that specified the reference trajectory are preprocessed by
transforming them to the vehicle's coordinate system. This simplifies the
calculations, because in the vehicle's coordinates the trajectory starts at
point (0, 0), with the orientation angle of 0. The transformations are
implemented in the `ReferenceTrajectory` class. 

## Model Predictive Control with latency

In this project, the simulator receives the actuations with the latency of 0.1
second. To account for latency, I do the following in the project: 

1. Using the dynamic model, current state, and current actuations, calculate the
corrected state with respect to the latency value;
2. Use the corrected state as the input to the optimizer. 

This approach is implemented in `MPC::ApplyLatency()` private method. 

Besides, I add a term to the cost function that punishes high speed values
at sharp steering angles. This addition had positive effect on the car's
behavior at curvy parts of the track. 

## Running in simulator

[![Alt text](https://img.youtube.com/vi/DIB1kMVHLRo/0.jpg)](https://www.youtube.com/watch?v=DIB1kMVHLRo)



