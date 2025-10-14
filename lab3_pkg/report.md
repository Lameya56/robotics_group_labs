# Lab 3 Report

## Plots & Visualizations

**Open-loop vs. odometry path**

![alt text](odometry_plot(recorded_vs_ideal_path)-1.png)

![alt text](image.png)

![alt text](image-1.png)

**PD trajectory vs. commanded path**

![alt text](image-3.png)

**Wall-following behavior**

![alt text](image-6.png)

**/odom vs. /odometry/filtered vs. /gazebo/model_states**

![alt text](image-5.png)

**RViz**

Part 2

![alt text](image-2.png)

## Tehnical Report
**Methods: how you implemented each part (logger, PD, PID, EKF).**

Logger: 

I implemented an odometry logger node (odom_logger.py) that subscribes to both /odom (for measured velocities) and /cmd_vel (for commanded velocities). The node logs timestamp, odom_lin_x, odom_ang_z, cmd_lin_x, and cmd_ang_z into a CSV file.

Each message callback stores the most recent readings in variables, and a timer periodically writes synchronized data rows to disk. This allowed for later comparison of commanded versus measured motion.

To visualize performance, I wrote plot_odom.py and compare_plot.py scripts using matplotlib to plot:
Odometry vs. Ideal Path (open-loop drift)
Commanded vs. Odometry Linear Velocity
Commanded vs. Odometry Angular Velocity

PD:

We implemented a PD Controller by first defining a node designed to follow a sequence of waypoints using pre-determined control parameters: Kp_lin, Kp_ang, Kd_ang, and goal_tol. These parameters determine how aggressively the controller responds to positional and angular errors. Specifically, we set Kp_lin to a relatively low value to reduce the likelihood of overshooting the target, allowing the robot to move more steadily toward each waypoint. We increased Kp_ang to enable sharper turns and faster angular corrections, helping the robot better align itself with the desired heading. Additionally, we raised Kd_ang to provide damping for angular motion, which helps smooth out oscillations and stabilize the turning response. The goal_tol parameter was kept small to ensure that the robot accurately reached each waypoint before proceeding to the next. By spacing the waypoints at short, regular intervals (increments of 0.2 units), we achieved a smoother overall trajectory since the smaller positional gaps required finer corrections and minimized overshoot, especially when navigating corners.

PID:
We first used the starter code to help us determine good starting values for the right-following wall code. Those values were: desired_distance, lookahead, theta_deg, Kp, Ki, and Kd. After testing with the initial defaulted values, we found that the robot was moving too erratically because it was moving towards the closest obstacle instead of the closest wall. All values had to be lowered significantly, as the desired_distance, lookahead, and theta_deg all combined to determine how quickly the robot would react to an obstacle. By repeatedly testing with RViZ2 and Gazebo, we fine-tuned the robot's calculations so that it would be an appropriate distance from the wall, while also taking smooth movements and rotations to complete a loop around the walls. We made sure to follow the ratio for the values Kp, Ki, and Kd so that the robot wouldn't overshoot and overcorrect its own movements.

EKF:

To implement the EKF, we began by configuring the frame parameters in the ekf.yaml file. We identified the appropriate frames by running the command
```
ros2 topic echo /odom --once | grep "frame_id"
```

which revealed that our base_frame differed from the one used in the example configuration. After adjusting the frames accordingly, we proceeded to integrate the remaining components of the EKF template. However, when we initially ran the implementation, the Ground Truth trend did not appear on the graph. Through debugging and research, we discovered that this issue occurred because the “gazebo/model_states” topic is no longer supported in ROS2. With assistance from AI, we resolved this by extracting Ground Truth data from the “/tf” topic instead. Specifically, we computed the Ground Truth coordinates by reading the transformation between the odom frame and the base_footprint frame.

**Results: trajectory plots, screenshots, performance observations.**
Please refer to the plots and graphs above.

Part 4: Open Ended Question

To calculate the average error for odometry and filtered odometry, we first measured the distance between each data point and the corresponding Ground Truth position, summed these distances (using the Euclidean distance formula), and divided by the total number of points. However, it’s important to note that our group’s average error values may not be fully accurate, as we used a workaround to obtain Ground Truth data, and the resulting graphs produced unexpected behavior.

However, with the data we have, we were able to find that:
Average Error for Odometry: 0.0513 meters
Average Error for Odometry Filtered: 0 meters

**Comparisons: open-loop vs. closed-loop, /odom vs. /odometry/filtered.**

In open-loop mode, the robot was driven using timed velocity commands to trace a square. The Odometry vs Ideal Path plot showed drift accumulating over each leg and how the square gradually became distorted. This is expected because open-loop control does not correct for wheel slip, latency, or noise in wheel encoder measurements.

**Controller tuning: how you chose/tuned Kp, Ki, Kd.**

First, I decreased the value of Kp, because the robot was moving too erratically. I noticed that the robot would move towards the obstacles in the middle instead of towards the right-hand wall, which meant I also had to decrease the Kd so there was less overturning. I initially had Ki at 0.0, and I had kept it at that value for the right-wall turning. I had to do a lot of trial and error and slowly increased/decreased each of the values. If the robot didn't rotate enough towards the wall, that meant I had to increment Kp and Kd accordingly. If the robot was overadjusting and oscillating quickly while turning, that meant I had to decrease Kp and Kd. For the left-turning, I used a small amount of Ki (0.01) because I noticed that for the left wall, the robot was more prone to turning too much; Ki helped correct some of the error over time. In addition to modifying these values, I flipped the geometry angles so one robot beam was facing left, the other beam was facing northwestward. I also flipped the values of getting the error, because I realized that when the error was positive, the robot would turn towards the right. When the value was negative, the robot would then move towards the left, which is what I needed for the left-turning modification. My end result isn't completely perfect, as the robot does sometimes oscillate back and forth to correct itself, but I managed to get the robot to make smooth turns around the turtle's perimeter.

**Challenges: problems encountered and how you solved them.**

The biggest challenge our team faced in this lab was getting the robot to accurately follow the commanded path while accounting for real-world sensor noise and odometry drift. Initially, our PD controller often overshot or failed to reach waypoints precisely, and the logged trajectories showed noticeable discrepancies between the commanded path, odometry readings, and Gazebo ground truth. We addressed this by carefully tuning the PD gains for both linear and angular velocities, adding a derivative term for angular control to reduce oscillations, and implementing a small slowdown when the robot needed to turn sharply. Logging odometry, filtered estimates, and ground truth helped us visually analyze errors and iteratively refine the controller and waypoint strategy.

Another challenge emerged in Part 4 when the graph failed to display the Ground Truth points. After extensive testing, we discovered that the template code was not fully compatible with ROS 2, as the Model_States topic was no longer supported. To address this, we modified the implementation to use the /tf topic for retrieving Ground Truth data, which allowed us to generate a graph displaying all three trends. However, the resulting trends did not exhibit the expected behavior—where the Ground Truth trend should represent the robot’s actual position, the Odometry (Filtered) trend should closely follow it, and the Odometry trend should appear slightly further away. Additionally, the graphs showed inconsistency across multiple runs; in some cases, certain trends were missing altogether. Even after resetting the devices, the issue persisted, preventing us from obtaining consistent and reliable results.

**Division of labor: who did what in the group (coding, plotting, writing, debugging, etc.). Confirmation that all members participated and understand the work.**
Lameya: Completed part 1 and worked on part 2 partially. 
Patrick: Completed part 2 and worked part 4 partially.
Lina: Completed part 3.

## Reflection Questions
**Part 1: Odometry and Open-Loop**

How did your robot’s actual odometry path compare to the commanded open-loop square? Where did you see drift or deviation? What factors caused the robot to deviate (simulation noise, wheel slip, etc.)?

The commanded open-loop path was a perfect square but the robot's actual odometry path deviates significantly from the square in several ways. The most obvious deviation is the rounded corners instead of sharp 90° turns, indicating that the robot executed turns with finite angular velocity rather than instantaneous rotations. The path also fails to close properly and does not return to the origin which demonstrates accumulated odometry drift. Additionally, the executed square is asymmetric and distorted, appearing wider than it is tall. These deviations likely stem from multiple factors including wheel slip during turns, odometry integration errors that compound over time and simulation-specific issues like added sensor noise. The cumulative effect shows how open-loop control without feedback correction leads to significant position errors, especially after multiple turns where small errors in each movement accumulate throughout the trajectory.

Why does dead-reckoning (open-loop odometry) accumulate error over time?

Dead-reckoning computes the robot’s position based solely on wheel encoder data and motion commands, without external correction. Each estimate depends on the previous one, so any small error in velocity, orientation, or distance traveled gets carried forward. Over time, these errors compound which causes the estimated position to drift further from the true location. Essentially, odometry errors integrate over time, leading to growing inaccuracy the longer the robot runs without correction from external sensors like LIDAR.

**Part 2: Closed-Loop PD Control**

What effect did changing Kp and Kd have on the robot’s ability to follow waypoints? Give an example of one tuning attempt that worked well and one that caused instability.

Adjusting Kp affected the robot’s responsiveness to heading errors. Increasing Kp allowed the robot to correct its heading more quickly along straight paths, but excessively high values caused overshooting and oscillations. For example:

- Low Kp (1.0): Angle turning becomes inaccurate and requires frequent self-correction.

- Very low Kp (0.1): Angle turning fails completely, causing the robot to circle instead of reaching waypoints.

- High Kp (100): Angle turning overshoots with significant oscillations.

Kd influenced the damping of corrections. Low Kd (0.01) made waypoint following less smooth, requiring more adjustments, while high Kd (100) caused heavy oscillations, disrupted waypoint tracking, and prevented proper turning at corners.

A good tuning example was Kp = 3.0 and Kd = 0.5, which allowed smooth trajectories and turns with minimal oscillations. Poor tuning examples include any of the extreme Kp or Kd values listed above, which caused the robot to deviate from waypoints or fail to navigate corners.

Why is angle wrap-around handling (constraining yaw error to (-\pi,\pi]) necessary in your controller?

Angle wrap-around handling is necessary in the controller because yaw angles are periodic and naturally wrap around at ±π. Without it, calculating the difference between the robot’s current heading and the goal can produce misleading results, causing the controller to command unnecessarily large rotations. For example, if the robot’s current yaw is near +pi and the goal is near -pi, a naive subtraction would indicate a rotation of almost 2π radians, even though the robot only needs to turn a small angle in the opposite direction. By constraining the yaw error to the interval (-pi,pi], the controller always computes the shortest rotation direction and magnitude, ensuring efficient, stable, and predictable motion toward the goal. This prevents overshooting, oscillations, and excessive angular commands, which are critical for accurate and smooth navigation.

**Part 3: PID Wall Following**

How does the lookahead distance L affect wall-following performance? What happens with very small vs. very large values?

The lookahead distance affects wall-following performance, because it affects how the robot oscillates in relation to incoming obstacles or walls. I noticed when the lookahead distance was very small, the robot would move erratically, meaning it would rotate back and forth to the point where it would spazz out and glitch around the environment. When the lookahead distance was very large, the robot would have very slow turning, to the point where it was not properly reacting to incoming walls. For me, the robot would end up crashing into the wall, and the RViZZ sensor would show some turning movement being attempted, but because it hit the wall, the robot began to drift off it's predicted course.

When would adding integral gain Ki be useful in wall following? Did you find you needed it in this lab? Why or why not?

Adding Ki would be useful in wall following, as it helps account for errors that cannot be corrected by modifying Kp and Kd. I used a very small amount (0.01) to help the robot navigate around the walls better for left-turning, because I noticed at the start and middle points of wrapping around the turtle, it would be a little far from the wall than intended. Once I added Ki, the robot adjusted better so that it was closer to the wall and made slightly better turns.

**Part 4: State Estimation (EKF)**

In your plots, how did /odometry/filtered compare to /odom and the Gazebo ground truth? Did the EKF reduce drift? Were there situations where the filter underperformed?

In our plots, /odometry/filtered generally followed the /odom data closely along straight segments of the robot’s path, and it appeared to reduce drift compared to raw odometry. However, during turns, the filter underperformed, showing less accurate corrections for angular motion. The /odometry/filtered trend almost completely overlapped with the Ground Truth, though this may have been influenced by the workaround we used to obtain Ground Truth data from the /tf topic. Overall, the EKF improved drift along straight paths but was less effective at handling rotational errors during turns.

Why is it beneficial to fuse wheel odometry and IMU, rather than relying on one sensor alone?

Fusing wheel odometry and IMU is advantageous because each sensor has inherent limitations when used alone. Wheel odometry is effective for measuring short-term displacement but accumulates error over time due to wheel slippage and encoder drift. IMU measurements, on the other hand, provide accurate rotational data and quick responsiveness but are inherently noisy, and integrating them over time can increase drift. By combining both sensors using an Extended Kalman Filter (EKF), the system produces a more reliable estimate of the robot’s pose, reducing both the drift seen in odometry and the noise present in raw IMU data.




**Group Reflection**

As a team, what was the biggest challenge you faced in this lab? How did you overcome it? What would you do differently if you had to repeat the project?

The biggest challenge in Part 4 was ensuring proper visualization of Ground Truth data. The original template was incompatible with ROS 2 due to the deprecated **Model_States** topic, and although switching to the **/tf** topic enabled graph generation, the resulting trends were inconsistent and did not accurately reflect the robot’s expected motion, preventing reliable results.

If we were to repeat this project, we would plan and start earlier to allow sufficient time to seek guidance and address challenges more effectively.

What did you learn about the importance of feedback control and state estimation in robotics?

This lab highlighted the critical importance of feedback control and state estimation in robotics. Feedback control allows the robot to correct for errors in real time, compensating for disturbances and imperfect actuators. State estimation, like the EKF, is essential for combining noisy sensor data into a coherent picture of the robot’s position and orientation. Without accurate state estimates, even a well-designed controller may fail to track the desired path. Together, they demonstrate how perception and control must work hand-in-hand for reliable autonomous navigation.
