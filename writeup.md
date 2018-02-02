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

Examples of these are given in the following video:

https://youtu.be/5pPcM6od6ko

### Description of Hyperparameter Choice


The hyperparameters were first tuned using the twiddle algorithm. This gave information about the general magnitude
of each hyperparameter. The final hyperparameters were then chosen by manual adjustment.
