.. _configuring_velocity_smoother:

Velocity Smoother
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_velocity_smoother

The ``nav2_velocity_smoother`` is a package containing a lifecycle-component node for smoothing velocities sent by Nav2 to robot controllers.
The aim of this package is to implement velocity, acceleration, and deadband smoothing from Nav2 to reduce wear-and-tear on robot motors and hardware controllers by smoothing out the accelerations/jerky movements that might be present with some local trajectory planners' control efforts.

See the package's README for more information.

Velocity Smoother Parameters
****************************

:smoothing_frequency:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         20.0
  ============== ===========================

  Description
    Frequency (Hz) to use the last received velocity command to smooth by velocity, acceleration, and deadband constraints. If set approximately to the rate of your local trajectory planner, it should smooth by acceleration constraints velocity commands. If set much higher, it will interpolate and provide a smooth set of commands to the hardware controller.

:scale_velocities:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    Whether or not to adjust other components of velocity proportionally to a component's required changes due to acceleration limits. This will try to adjust all components to follow the same direction, but still enforces acceleration limits to guarantee compliance, even if it means deviating off commanded trajectory slightly.

:feedback:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  string         "OPEN_LOOP"
  ============== ===========================

  Description
    Type of feedback to use for the current state of the robot's velocity. In ``OPEN_LOOP``, it will use the last commanded velocity as the next iteration's current velocity. When acceleration limits are set appropriately, this is a good assumption. In ``CLOSED_LOOP``, it will use the odometry from the ``odom`` topic to estimate the robot's current speed. In closed loop mode, it is important that the odometry is high rate and low latency, relative to the smoothing frequency.

:max_velocity:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  vector<double> [0.5, 0.0, 2.5]
  ============== ===========================

  Description
    Maximum velocities (m/s) in ``[x, y, theta]`` axes.

:min_velocity:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  vector<double> [-0.5, 0.0, -2.5]
  ============== ===========================

  Description
    Minimum velocities (m/s) in ``[x, y, theta]`` axes. This is **signed** and thus must be **negative** to reverse. Note: rotational velocities negative direction is a right-hand turn, so this should always be negative regardless of reversing preference.

:deadband_velocity:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  vector<double> [0.0, 0.0, 0.0]
  ============== ===========================

  Description
    Minimum velocities (m/s) to send to the robot hardware controllers, to prevent small commands from damaging hardware controllers if that speed cannot be achieved due to stall torque.

:velocity_timeout:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.0
  ============== ===========================

  Description
    Timeout (s) after which the velocity smoother will send a zero-ed out ``Twist`` command and stop publishing.

:max_accel:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  vector<double> [2.5, 0.0, 3.2]
  ============== ===========================

  Description
    Maximum acceleration to apply to each axis ``[x, y, theta]``.

:max_decel:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  vector<double> [-2.5, 0.0, -3.2]
  ============== ===========================

  Description
    Minimum acceleration to apply to each axis ``[x, y, theta]``. This is **signed** and thus these should generally all be **negative**.

:odom_topic:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  string         "odom"
  ============== ===========================

  Description
    Topic to find robot odometry, if in ``CLOSED_LOOP`` operational mode.

:odom_duration:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.1
  ============== ===========================

  Description
    Time (s) to buffer odometry commands to estimate the robot speed, if in ``CLOSED_LOOP`` operational mode.

Example
*******
.. code-block:: yaml

  velocity_smoother:
    ros__parameters:
      smoothing_frequency: 20.0
      scale_velocities: false
      feedback: "OPEN_LOOP"
      max_velocity: [0.5, 0.0, 2.5]
      min_velocity: [-0.5, 0.0, -2.5]
      deadband_velocity: [0.0, 0.0, 0.0]
      velocity_timeout: 1.0
      max_accel: [2.5, 0.0, 3.2]
      max_decel: [-2.5, 0.0, -3.2]
      odom_topic: "odom"
      odom_duration: 0.1
