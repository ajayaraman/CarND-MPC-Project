# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program Model Predictive Control Project

### Instructions to build and run

For building the project refer to instructions [here](https://github.com/udacity/CarND-MPC-Project)

To run the project use

```
./mpc 
```

### Final result

Link to youtube video of final simulation result

[![MPC Control](https://img.youtube.com/vi/z2E3JeLZnbU/0.jpg)](https://youtu.be/z2E3JeLZnbU)

### Discussion on MPC parameter selection

#### Choice of time horizon and impact on N and dt
To properly steer a vehicle, the time horizon must not be too small or else the vehicle will not be able to optimize for a larger trajectory. However, the converse was also found to be true in practice. We cannot expect the
controller to perform too well when the time horizon to optimize over is too large because our model's kinematics are an approximation that holds well for the immediate few seconds and not more. 

For this project, I found that a time horizon of t = 2 seconds was reasonable by looking at the length of the green-line which represents the MPC controller's predicted trajectory with optimal controls applied to our kinematic model.

Once that time horizon was selected, I started with N = 10, dt = 0.2 but found that the predictions were not accurate enough. So I opted for predicting over smaller dt = 0.1 seconds and increasing N to 20 steps. This proved sufficient for lower speeds of upto 50 kph.

#### Choice of cost function 
The cost function has multiple parts. I prioritized tracking the trajectory over speed by using a 2000:1 factor which allowed the vehicle to travel at a slower speed. I also prioritized the smoothness of the successive control actions by weighting it with a factor of 500 for the steering and 100 for throttle. These values were iteratively arrived at through manual trial and error. 

#### Accounting for Latency

In order to account for the 100 ms latency, the MPC controller is initialized with state computed with a delay of 100ms provided the present controls are applied using the kinematic model of actuation.

#### Further Improvements
A better method to tune the cost function will be to run a series of simulations in parallel in a monte-carlo fashion to try out all possible values in a range or use twiddle/bayes-optimization to evalute the direction to move in order to best minimize the RMSE error with ground-truth locations where the vehicle is expected to be for the given scenario.