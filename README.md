# micropython_sensorfusion

The objective - Leverage micropython (essentially python for microcontrollers) for some real prototyping work - 'Compare sensor fusion filters/algorithms'.

The idea was to assess - how easy is it to get up and running and in something beyond that of the typical embedded 'hello world' example.

# My set-up:
  ESP32 board running micropython
  Jupyter Notebooks for flashing, debugging your code via the repl
  IMU (mpu6050) for my gyro, accelerometer readings
  Processing IDE for visualization
  
# The Goal:
  Read raw values from the IMU's onboard (MEMS) gyroscope and accelerometer
  Do some math to convert raw readings - angular velocity and linear acceleration to actual degrees of rotation about a particular axis.
  Use the computed angles to simulate 3D rotation in the processing IDE.

# The problem:
  Accelerometer readings are pretty noisy but average out over longer timescales
  Gyroscope readings are pretty good over short durations but tend to drift due to integration of the 'bias error' (noise in sensor readings).

The problem of noisy readings can be addressed with 'sensor fusion' filters which involves the application of some control theory to fuse readings from multiple sensors. Without going into the nitty gritty technical details, 2 good ones are.

  Complementary filter: This filter combines two sensor readings together in a way that produces the best
  estimate. The filter is as it simple as it gets (can be implemented as a single line of code).
  In our case, this filter does exactly what is necessary, it favors the gyroscope readings for short time durations and the accelerometer's average readings over a longer time duration. This is done by arriving at a cutoff % for trusting each sensor's measurement.
  In my case, cutoff is defined to be 0.90. This means that at every time step, 90% of the new angle measurement comes from the old angle measurement plus the integrated gyro measurement. The remaining 10% comes from the accelerometer. This slowly averages in the accelerometer over many time steps.

  Kalman filter: This one's a little more complex in terms of its implementation but follows the same logic - its great for dealing with sensor noise.
  We build a model of the system as a couple of equations (i.e. probability distribution for the state of the system).
  Use this model to estimate or predict the next measurement which is again a probability distribution
  And lastly use our predicted measurement to balance/update the (actual) observed measurement.

The equations might look whacky but the theory is pretty intuitive. Here is a lovely write-up on the subject - https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

# Usage:

Simply clone the repository and follow these steps
  Wire up the sensors and board
  Flash the code onto the board 
  Install the processing IDE
  Use the included processing sketch to visualize a 3D simulation.
