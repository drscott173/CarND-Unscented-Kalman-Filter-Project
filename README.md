# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

test

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

5. There are several options:
```
-h help
-a <float> standard deviation in acceleration when driving straight
-A <float> standard deviation in acceleration when turning
-l only use laser, e.g., turn off radar
-r only use radar, e.g., turn off laser
-s search for optimal acceleration values
```

## Results

## Sample 1

Use the following command:

```
% ./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt out1.txt
Accuracy - RMSE:
 0.038556
0.0443807
 0.483245
 0.502898
Valid NIS: 65.5229%
Done!

%
```
We interpret the output of out1.txt below.

The figure below shows how the position varies over time, forming
a figure 8.  The actual positions are shown in blue and our predictions
are shown in red.

![Position 1](images/position_1.png?raw=true "Predicting Position 1")

The figure below shows how the velocity varies over time, jumping
from a standing start to jerky variations between 2 and 4 meters
per second. Our prediction has high frequency noise at first, but quickly
dampens down, creating a model that uses the a value closer to the
mean velocity to predict position.

![Velocity 1](images/velocity_1.png?raw=true "Predicting Velocity 1")

Finally, the figure below shows our NIS values for the chi square distribution.
We want most of our values to be below 7.8 according to the chi square table, 
which means most of the measurements would fall within our model's predicted area
of uncertainty.

Here's how to read this image.  The green line on top represents variations in
the Y position through time.  The blue line below repreasents variations in the X
position through time.  Our vehicle is essentially oscillating back in both X and Y,
forming a figure 8.

The model spikes above our 95% prediction line when we have abrupt changes in 
X or Y direction.  That's because our model prefers continuous motion in one 
direction, assuming smooth acceleration.  When we change direction, the model
overshoots or undershoots.  The actual measurement falls outside our predicted
area as a result.

![NIS 1](images/nis_1.png?raw=true "NIS Consistency 1")

## Sample 2

Execute the following:

```
% ./UnscentedKF ../data/sample-laser-radar-measurement-data-2.txt out2.txt
Accuracy - RMSE:
0.154763
0.186962
0.292497
0.428491
Valid NIS: 77.7778% 
Done!

%```

As before, we show the position of our object over time in blue.  Our
predicted position is in red.  The model nails it.

![Position 2](images/position_2.png?raw=true "Predicting Position 1")

The object in this sample has a velocity that first jumps from a standing start,
then oscillates in a sine wave. The actual velocity is in blue, our predictions
are in red.  We can see that our model has an initial impulse and assumes the
object will keep accelerating quickly from that jumpstart. That doesn't happen,
of course, so our model tamps down.  When velocity changes direction our model
introduces noise as it adjusts.

![Velocity 2](images/velocity_2.png?raw=true "Predicting Velocity 1")

Below we see a measure of consistency.  The model has a bit of rough time at
start, as it assumes a standing start when velocity numbers start arriving.  After
a few cycles the model becomes consistent and accurately covers the uncertainty
in the object's motion.  The few times it misses are when the velocity changes
direction.

![NIS 2](images/nis_2.png?raw=true "NIS Consistency 1")

## Discussion

