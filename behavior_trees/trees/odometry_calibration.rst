.. _behavior_tree_odometry_calibration:

Odometry Calibration
####################

This behavior tree drives the robot in a CCW square using the DriveOnHeading and Spin behaviors.


.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Repeat num_cycles="3">
        <Sequence name="Drive in a square">
          <DriveOnHeading dist_to_travel="2.0" speed="0.1" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.1" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.1" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
          <DriveOnHeading dist_to_travel="2.0" speed="0.1" time_allowance="12"/>
          <Spin spin_dist="1.570796" is_recovery="false"/>
        </Sequence>
      </Repeat>
    </BehaviorTree>
  </root>
