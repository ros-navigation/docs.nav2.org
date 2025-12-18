.. _symmetric_yaw_tolerance_guide:

Symmetric Yaw Tolerance for Goal Checking and Navigation
#########################################################

Overview
********

The **symmetric_yaw_tolerance** parameter enables symmetric robots (that can drive equally well in both forward and backward directions) to reach goals without unnecessary 180° rotations. This feature is available in:

- **GoalAngleCritic** (MPPI Controller) - for trajectory cost evaluation
- **SimpleGoalChecker** (Controller Server) - for goal achievement detection

When enabled, these plugins accept either the goal orientation or the goal orientation + 180°, preventing the robot from wasting time and energy rotating when it could simply drive backward.

Use Case
********

This feature is ideal for robots with symmetric mechanical designs, such as:

- Differential drive robots with sensors on both ends
- Robots with bidirectional capabilities

Without this feature, a standard goal checker or goal angle critic would force the robot to rotate 180° if it approaches the goal from the "wrong" direction, even when the robot could simply drive backward to the goal.

How It Works
************

When ``symmetric_yaw_tolerance: true`` is set:

**In GoalAngleCritic (MPPI Controller):**
  The critic calculates the angular distance to both the goal orientation and the flipped goal orientation (goal + 180°), then uses the minimum of these two distances for trajectory scoring. This allows trajectories approaching from either direction to have lower costs.

**In SimpleGoalChecker (Controller Server):**
  The goal checker returns true if the robot is within tolerance of either orientation - the exact goal orientation OR the goal orientation + 180°.

This allows the robot to:

1. Approach the goal from either direction without penalty
2. Avoid unnecessary rotations when already facing away from the goal orientation
3. Select the most efficient trajectory based on current orientation
4. Accept goal achievement when facing backward (goal ± 180°)

Enabling the Feature
********************

To enable symmetric yaw tolerance for your robot:

**In the Goal Checker:**

Add ``symmetric_yaw_tolerance: true`` to your SimpleGoalChecker configuration:

.. code-block:: yaml

    controller_server:
      ros__parameters:
        goal_checker_plugins: ["goal_checker"]
        goal_checker:
          plugin: "nav2_controller::SimpleGoalChecker"
          xy_goal_tolerance: 0.15
          yaw_goal_tolerance: 0.15
          symmetric_yaw_tolerance: true

**In the MPPI Controller:**

Add ``symmetric_yaw_tolerance: true`` to your GoalAngleCritic configuration:

.. code-block:: yaml

    controller_server:
      ros__parameters:
        FollowPath:
          plugin: "nav2_mppi_controller::MPPIController"

          critics:
            - "GoalAngleCritic"

          GoalAngleCritic:
            cost_weight: 5.0
            cost_power: 1
            threshold_to_consider: 0.4
            symmetric_yaw_tolerance: true

**Complete Example:**

.. code-block:: yaml

    controller_server:
      ros__parameters:
        use_sim_time: false
        controller_frequency: 30.0

        # Goal checker with symmetric support
        goal_checker_plugins: ["goal_checker"]
        goal_checker:
          plugin: "nav2_controller::SimpleGoalChecker"
          xy_goal_tolerance: 0.15
          yaw_goal_tolerance: 0.15
          stateful: true
          symmetric_yaw_tolerance: true

        # Controller with symmetric support
        controller_plugins: ["FollowPath"]
        FollowPath:
          plugin: "nav2_mppi_controller::MPPIController"
          time_steps: 56
          model_dt: 0.05
          batch_size: 2000
          motion_model: "DiffDrive"

          critics:
            - "ConstraintCritic"
            - "GoalCritic"
            - "GoalAngleCritic"
            - "PathAlignCritic"

          ConstraintCritic:
            cost_weight: 4.0
          GoalCritic:
            cost_weight: 5.0
            threshold_to_consider: 1.4
          GoalAngleCritic:
            cost_weight: 5.0
            threshold_to_consider: 0.4
            symmetric_yaw_tolerance: true
          PathAlignCritic:
            cost_weight: 10.0

Configuration Options
*********************

Both plugins can be configured independently. You can enable one, the other, or both depending on your needs:

**Only Goal Checker Enabled:**
  The robot accepts goals as reached when facing either direction (goal or goal ± 180°), but trajectory planning doesn't prefer the backward approach. Useful if you want goal flexibility without trajectory planning changes.

**Only Goal Angle Critic Enabled:**
  The trajectory planner prefers minimal rotation and approaches that minimize angle difference, but the goal is only marked as reached in the exact orientation. Useful for partial trajectory optimization.

**Both Enabled:**
  Complete symmetric support - trajectory planning prefers efficient approach AND goal acceptance is orientation-agnostic. **Recommended** for full symmetric robot support.

**Expected Behavior:**

- With ``symmetric_yaw_tolerance: true``, the robot should accept goals when approaching from either direction
- GoalAngleCritic should prefer minimal rotation when within threshold_to_consider
- Robot should achieve goal without unnecessary 180° rotations

Related Documentation
*********************

- :ref:`configuring_mppic` - MPPI Controller configuration guide
- :ref:`configuring_controller_server` - Controller Server configuration guide
- `Nav2 MPPI Controller Documentation <https://navigation.ros.org/configuration/packages/configuring-mppi.html>`_
- `Nav2 Goal Checker Plugins <https://navigation.ros.org/configuration/packages/configuring-controller-server.html#goal-checker-plugins>`_
