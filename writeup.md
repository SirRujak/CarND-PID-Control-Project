# PID Control of a Simulated Vehicle

## Compilation

This code requires the installation of the uWebSockets library found here:

```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
```


The code in this repository comiples through the following process:

1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./pid.


## Implementation

The code implements a classical PID controller with hyperparameter tuning based on the twiddle algorithm followed
by manual configuration.

## Reflection

### Description of PID Elements

Each portion of the PID controller was examined as to its effect on the total system.

1. The proportional term allows the system to understand its current error from a given position.
Without this term the system is unable to correct in the right direction at a high enough speed
and eventually loses control of the vehicle.

2. The integral term of the system ensures that the vehicle takes corners smoothly. Without this term the system
takes too long to correct for increasing error.

3. The derivative poriton of the system dampens oscillations. Without this term the system can fall into
self reinforcing oscillations that will eventually run the vehicle off the road.

These results are consistent with expected values.

Here is a general explanation of PID controllers:

#### Proportional

A proportional controller will use a given baseline to calculate the current error from a given goal. This error is then
multiplied by a set factor such that the response of a given proportional controller will react proportionally to the error.

#### Integral

A PI controller will also take the integral of recent errors in order to account for sudden large changes in the error.
Recent large errors will help the PI react proportional to both the current error and historical errors. A side effect of
adding an integrational factor to these controllers is that it is possible to account for system wide errors alongside
actuator errors.

#### Derivative

A PID controller uses the derivative of the current error change in order to dampen oscillations. This element is indifferent
to current error and only attempts to stabalize the change in error over time.

Examples of these are given in the following video:

https://youtu.be/5pPcM6od6ko

### Description of Hyperparameter Choice


The hyperparameters were first tuned using the twiddle algorithm. This gave information about the general magnitude
of each hyperparameter. The final hyperparameters were then chosen by manual adjustment.
