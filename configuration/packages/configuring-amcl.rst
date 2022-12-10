.. _configuring_amcl:

AMCL
####

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl

AMCL implements the server for taking a static map and localizing the robot within it using an Adaptive Monte-Carlo Localizer.

Parameters
**********

:alpha1:

  ============== =======
  Type           Default
  -------------- -------
  double         0.2   
  ============== =======

  Description
    Expected process noise in odometry's rotation estimate from rotation.

:alpha2:

  ============== ==============
  Type           Default                                               
  -------------- --------------
  double         0.2   
  ============== ==============

  Description
    Expected process noise in odometry's rotation estimate from translation.

:alpha3:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.2   
  ============== =============================

  Description
    Expected process noise in odometry's translation estimate from translation.

:alpha4:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.2   
  ============== =============================

  Description
    Expected process noise in odometry's translation estimate from rotation.

:alpha5:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.2   
  ============== =============================

  Description
    For Omni models only: translation noise.

:base_frame_id:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "base_footprint"            
  ============== =============================

  Description
    Robot base frame.

:beam_skip_distance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5            
  ============== =============================

  Description
    Ignore beams that most particles disagree with in Likelihood field model. Maximum distance to consider skipping for (m).

:beam_skip_error_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.9         
  ============== =============================

  Description
    Percentage of beams after not matching map to force full update due to bad convergance.

:beam_skip_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.3         
  ============== =============================

  Description
    Percentage of beams required to skip.

:do_beamskip:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           False         
  ============== =============================

  Description
    Whether to do beam skipping in Likelihood field model.

:global_frame_id:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "map"         
  ============== =============================

  Description
    The name of the coordinate frame published by the localization system.

:lambda_short:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.1         
  ============== =============================

  Description
    Exponential decay parameter for z_short part of model.

:laser_likelihood_max_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         2.0         
  ============== =============================

  Description
    Maximum distance to do obstacle inflation on map, for use in likelihood_field model.

:laser_max_range:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         100.0         
  ============== =============================

  Description
    Maximum scan range to be considered, -1.0 will cause the laser's reported maximum range to be used.

:laser_min_range:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         -1.0         
  ============== =============================

  Description
    Minimum scan range to be considered, -1.0 will cause the laser's reported minimum range to be used.

:laser_model_type:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "likelihood_field"         
  ============== =============================

  Description
    Which model to use, either beam, likelihood_field, or likelihood_field_prob. Same as likelihood_field but incorporates the beamskip feature, if enabled.

:set_initial_pose:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           False         
  ============== =============================

  Description
    Causes AMCL to set initial pose from the initial_pose* parameters instead of waiting for the initial_pose message.

:initial_pose:

  ============== ==================================
  Type           Default                           
  -------------- ----------------------------------
  Pose2D         {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
  ============== ==================================

  Description
    X, Y, Z, and yaw coordinates of initial pose (meters and radians) of robot base frame in global frame.

:max_beams:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            60         
  ============== =============================

  Description
    How many evenly-spaced beams in each scan to be used when updating the filter.

:max_particles:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            2000         
  ============== =============================

  Description
    Maximum allowed number of particles.

:min_particles:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            500         
  ============== =============================

  Description
    Minimum allowed number of particles.

:odom_frame_id:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "odom"         
  ============== =============================

  Description
    Which frame to use for odometry.

:pf_err:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.05         
  ============== =============================

  Description
    Particle Filter population error.

:pf_z:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.99         
  ============== =============================

  Description
    Particle filter population density.

:recovery_alpha_fast:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0         
  ============== =============================

  Description
    Exponential decay rate for the fast average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.1.

:recovery_alpha_slow:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0         
  ============== =============================

  Description
    Exponential decay rate for the slow average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.001.


:resample_interval:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            1         
  ============== =============================

  Description
    Number of filter updates required before resampling.

:robot_model_type:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "nav2_amcl::DifferentialMotionModel"         
  ============== =============================

  Description
    The fully-qualified type of the plugin class. Options are "nav2_amcl::DifferentialMotionModel" and "nav2_amcl::OmniMotionModel". Users can also provide their own custom motion model plugin type.

  Note for users of galactic and earlier
    The models are selectable by string key (valid options: "differential", "omnidirectional") rather than plugins.

:save_pose_rate:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5         
  ============== =============================

  Description
    Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used on subsequent runs to initialize the filter (-1.0 to disable).

:sigma_hit:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.2         
  ============== =============================

  Description
    Standard deviation for Gaussian model used in z_hit part of the model.

:tf_broadcast:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           True         
  ============== =============================

  Description
    Set this to false to prevent amcl from publishing the transform between the global frame and the odometry frame.

:transform_tolerance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0         
  ============== =============================

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future.

:update_min_a:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.2         
  ============== =============================

  Description
    Rotational movement required before performing a filter update.

:update_min_d:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.25         
  ============== =============================

  Description
    Translational movement required before performing a filter update.

:z_hit:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5         
  ============== =============================

  Description
    Mixture weight for z_hit part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand..

:z_max:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.05         
  ============== =============================

  Description
    Mixture weight for z_max part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand.

:z_rand:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5         
  ============== =============================

  Description
    Mixture weight for z_rand part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand..

:z_short:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.005         
  ============== =============================

  Description
    Mixture weight for z_short part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand.

:always_reset_initial_pose:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           False         
  ============== =============================

  Description
    Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter (with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the last known pose to initialize.
    
:scan_topic:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         scan         
  ============== =============================

  Description
    Laser scan topic to subscribe to.

:map_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         map
  ============== =============================

  Description
    Map topic to subscribe to.

:first_map_only:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           False
  ============== =============================

  Description
    Allows AMCL to accept maps more than once on the map_topic. This is especially useful when you're using the `LoadMap` service in `map_server`. Prior to Humble, this is ``first_map_only_``.


Example
*******
.. code-block:: yaml

    amcl:
      ros__parameters:
        alpha1: 0.2
        alpha2: 0.2
        alpha3: 0.2
        alpha4: 0.2
        alpha5: 0.2
        base_frame_id: "base_footprint"
        beam_skip_distance: 0.5
        beam_skip_error_threshold: 0.9
        beam_skip_threshold: 0.3
        do_beamskip: false
        global_frame_id: "map"
        lambda_short: 0.1
        laser_likelihood_max_dist: 2.0
        laser_max_range: 100.0
        laser_min_range: -1.0
        laser_model_type: "likelihood_field"
        max_beams: 60
        max_particles: 2000
        min_particles: 500
        odom_frame_id: "odom"
        pf_err: 0.05
        pf_z: 0.99
        recovery_alpha_fast: 0.0
        recovery_alpha_slow: 0.0
        resample_interval: 1
        robot_model_type: "nav2_amcl::DifferentialMotionModel"
        save_pose_rate: 0.5
        sigma_hit: 0.2
        tf_broadcast: true
        transform_tolerance: 1.0
        update_min_a: 0.2
        update_min_d: 0.25
        z_hit: 0.5
        z_max: 0.05
        z_rand: 0.5
        z_short: 0.05
        scan_topic: scan
        map_topic: map
        set_initial_pose: false
        always_reset_initial_pose: false
        first_map_only: false
        initial_pose:
          x: 0.0
          y: 0.0
          z: 0.0
          yaw: 0.0
