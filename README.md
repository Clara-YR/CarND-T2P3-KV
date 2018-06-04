[//]: # (Image References)

[image0]: ./ReadmeImages/ParticleFilterAlgorithmFlowchar.png "Flowchar"
[image1]: ./ReadmeImages/UpdateEquations.png "yaw_rate!=0"
[image2]: ./ReadmeImages/Transformation.png 
[image3]: ./ReadmeImages/ParticleWeights.png 


# Particle Filter


![alt text][image0]

## Initialization

For implementation in the particle filter project, I initialized my particle filter by sampling from a __Gaussian distribution__, taking into account:

- Gaussian sensor noise around initial GPS position estimate
- the initial heading estimate.

Use the 2nd method which is to sample around some kind of initial estimate. For a self-driving car, this initial estimate can come from a global positioning system, or GPS.

Although GPS has low accuracy and reduced availability in certain environments, it can be very useful to provide initial rough estimate of location.

[`std::normal_distribution`](http://en.cppreference.com/w/cpp/numeric/random/normal_distribution)

[`std::default_random_engine`](http://www.cplusplus.com/reference/random/default_random_engine/)

## Prediction Step

For the prediction step, I used what I've learned in the motion models lesson to predict where the car will be at the next time step.

For each particle, I had to update the particle's location based on velocity and yaw rate measurements.

To account for the uncertainty in the control input, I also added Gaussian noise to the velocity and yaw rate.

**Equations**
The equations for updating x, y and the yaw angle when the yaw rate is not equal to zero:

![alt text][image1]


## Update Step

Ultimate goal: determine how well each of our particle positions represents the actual position of our car, using car sersor and map input to weigh.

For a single particle:

- TRANSFORM  
- ASSCOCIATE
- UPDATE WEIGHTS: 
	- Determine measurement probabilities
	- Combine probabilities
	
### Transform

Observations in the car coordinate system can be transformed into map coordinates (x_m,y_m) by passing car observation coordinates (x_c,y_c), map particle coordinates (x_p,y_p), and our rotation angle (-90 degrees) through a homogenous transformation matrix, which performs rotation and translation.

![alt text][image2]


Transform observations to map coordinate
function `homogeneous transformation` is composed of a rotation in a translation
Essentiallly, 

- ORIENTATION: rotate the maps frame to match the particles point of view.
- ORIGIN: move the origin of the f
rame to the location of the particle

### Association
Data association is matching landmark measurements(observations) to objects in the real world(map landmarks). 

To pick the right measurement(observation) use the Nearest Neighor. Often times, you have some map landmarks that have multiple lidar measurements that could correspond to it. 


**Landmarks**

```
import numpy as np

# define coordinates and theta
x_part= 4
y_part= 5
x_obs= 0
y_obs= -4
theta= -np.pi/2 # -90 degrees

# transform to map x coordinate
x_map= x_part + (np.cos(theta) * x_obs) - (np.sin(theta) * y_obs)

# transform to map y coordinate
y_map= y_part + (np.sin(theta) * x_obs) + (np.cos(theta) * y_obs)

print(int(x_map), int(y_map)) # (0,5)
```

**Association**

associate each transformed observation with a land mark identifier.

**Particle Weights**

We can calculate particle weights using the following equation:

![alt text][image2]

Multivariate-Gaussian Probability

```
import math

# define inputs
sig_x= 0.3
sig_y= 0.3
x_obs= 6
y_obs= 3
mu_x= 5
mu_y= 3

# calculate normalization term
gauss_norm= (1/(2 * np.pi * sig_x * sig_y))

# calculate exponent
exponent= ((x_obs - mu_x)**2)/(2 * sig_x**2) + ((y_obs - mu_y)**2)/(2 * sig_y**2)

# calculate weight using normalization terms and exponent
weight= gauss_norm * math.exp(-exponent)

print(weight) # should be around 0.00683644777551 rounding to 6.84E-3
```

	

## Resample

After I updated the weights for each particle, I had to resample the particles with probability proportional to these weights.

Project Code

- `main.cpp` runs particle filter as well as measures its runtime and calculate the weighted error at each time step.

- `particle_filter.cpp` contains all of the implementations of the functions of the particle filter class.
	- `init` takes as input a GPS position (x, y) and an initial estimate (theta) and an array of uncertainties for these measurements (std[])
