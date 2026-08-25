.. _behavior_tree_odometry_calibration:

Odometry Calibration
####################

This behavior tree drives the robot in a CCW square three times using the DriveOnHeading and Spin behaviors.
The robot will traverse each side of the square at 0.2 (m/s) for 2 meters before making a 90 degree turn.
This is a primitive experiment to measure odometric accuracy and can be used and repeated to tune parameters related to odometry to improve quality.

.. image:: gifs/odometry_calibration.gif
  :width: 800
  :alt: Alternative text
  :align: center


.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Repeat num_cycles="3">
        <Sequence name="Drive in a square">
          <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
        </Sequence>
      </Repeat>
    </BehaviorTree>
  </root>
