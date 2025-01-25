.. _sdf_handson:

Setting Up The SDF - Gazebo
###########################

.. note:: Note that you have already setup a URDF in the previous tutorials but will also need to setup a SDF. URDF is used to set up the robot frames and describe the robot's structure for run-time use on hardware and possibly in simulation. SDF is a specific file for simulators, like Gazebo, that describes the simulator environment, model (including its frames and Gazebo-specific information), and appropriate plugins. The SDF that we will make is for Gazebo, but could be replaced with an appropriate SDF or other format file for Open3D Engine or Isaac Sim.

For this guide, we will be creating the SDF (Simulation Description Format) file for a simple differential drive robot to give you hands-on experience on building an SDF file for Gazebo simulation.

.. seealso::
  The complete source code in this tutorial can be found in `navigation2_tutorials <https://github.com/ros-navigation/navigation2_tutorials/tree/master/sam_bot_description>`_ repository under the ``sam_bot_description`` package. Note that the repository contains the full code after accomplishing all the tutorials in this guide.

About SDF
=========

SDF is a specific file for simulators, like Gazebo, that describes the simulator environment, model (including its frames and Gazebo-specific information), and appropriate plugins. The SDF that we will make is for Gazebo, but could be replaced with an appropriate SDF or other format file for Open3D Engine or Isaac Sim.

We can also use our SDF with the robot_state_publisher but the following package would be required to do that:

.. code-block:: shell

  sudo apt install ros-<ros2-distro>-sdformat-urdf

This package contains a C++ library and urdf_parser_plugin for converting SDFormat XML into URDF C++ structures. Installing it allows one to use SDFormat XML instead of URDF XML as a robot description.

.. seealso::
  If you want to learn more about the SDF and sdformat_urdf, we encourage you to have a look at the official `SDFormat Website <http://sdformat.org/>`__ and the `sdformat_urdf GitHub repository <https://github.com/ros/sdformat_urdf/tree/rolling/sdformat_urdf>`__

Writing the SDF
================

For now the SDF will pretty much be a copy of the URDF code converted into SDFormat. Changes between the two descriptions will happen when we start adding various plugins and sensors in the next tutorials.

Here is the SDF version of the URDF code:

.. code-block:: xml

  <?xml version="1.0" ?>
  <sdf version="1.8" xmlns:xacro="http://ros.org/wiki/xacro">
    <model name='sam_bot' canonical_link='base_link'>

      <!-- Define robot constants -->
      <xacro:property name="base_width" value="0.31"/>
      <xacro:property name="base_length" value="0.42"/>
      <xacro:property name="base_height" value="0.18"/>

      <xacro:property name="wheel_radius" value="0.10"/>
      <xacro:property name="wheel_width" value="0.04"/>
      <xacro:property name="wheel_ygap" value="0.025"/>
      <xacro:property name="wheel_zoff" value="0.05"/>
      <xacro:property name="wheel_xoff" value="0.12"/>

      <xacro:property name="caster_xoff" value="0.14"/>

      <!-- Define some commonly used inertial properties  -->
      <xacro:macro name="box_inertia" params="m w h d">
        <inertial>
          <pose>0 0 0 ${pi/2} 0 ${pi/2}</pose>
          <mass>${m}</mass>
          <inertia>
            <ixx>${(m/12) * (h*h + d*d)}</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>${(m/12) * (w*w + d*d)}</iyy>
            <iyz>0.0</iyz>
            <izz>${(m/12) * (w*w + h*h)}</izz>
          </inertia>
        </inertial>
      </xacro:macro>

      <xacro:macro name="cylinder_inertia" params="m r h">
        <inertial>
          <pose>0 0 0 ${pi/2} 0 0</pose>
          <mass>${m}</mass>
          <inertia>
            <ixx>${(m/12) * (3*r*r + h*h)}</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>${(m/12) * (3*r*r + h*h)}</iyy>
            <iyz>0.0</iyz>
            <izz>${(m/2) * (r*r)}</izz>
          </inertia>
        </inertial>
      </xacro:macro>


      <xacro:macro name="sphere_inertia" params="m r">
        <inertial>
          <mass>${m}</mass>
          <inertia>
            <ixx>${(2/5) * m * (r*r)}</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>${(2/5) * m * (r*r)}</iyy>
            <iyz>0.0</iyz>
            <izz>${(2/5) * m * (r*r)}</izz>
          </inertia>
        </inertial>
      </xacro:macro>

      <!-- Robot Base -->
      <link name='base_link'>
        <must_be_base_link>true</must_be_base_link>
        <visual name="base_link_visual">
          <geometry>
            <box><size>
              ${base_length} ${base_width} ${base_height}
            </size></box>
          </geometry>
          <material>
            <ambient>0 1 1 1</ambient>
            <diffuse>0 1 1 1</diffuse>
          </material>
        </visual>

        <collision name="base_link_collision">
          <geometry>
            <box><size>
              ${base_length} ${base_width} ${base_height}
            </size></box>
          </geometry>
        </collision>

        <xacro:box_inertia m="15" w="${base_width}" d="${base_length}" h="${base_height}"/>
      </link>

      <!-- Robot Footprint -->
      <link name='base_footprint'>
        <pose relative_to="base_joint"/>
        <xacro:box_inertia m="0" w="0" d="0" h="0"/>
      </link>

      <joint name='base_joint' type='fixed'>
        <parent>base_link</parent>
        <child>base_footprint</child>
        <pose relative_to="base_link">0.0 0.0 ${-(wheel_radius+wheel_zoff)} 0 0 0</pose>
      </joint>


      <!-- Wheels -->
      <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
        <link name="${prefix}_link">
          <pose relative_to="${prefix}_joint"/>

          <visual name="${prefix}_link_visual">
            <pose relative_to="${prefix}_link">0 0 0 ${pi/2} 0 0</pose>
            <geometry>
              <cylinder>
                <radius>${wheel_radius}</radius>
                <length>${wheel_width}</length>
              </cylinder>
            </geometry>
            <material>
              <ambient>0.3 0.3 0.3 1.0</ambient>
              <diffuse>0.7 0.7 0.7 1.0</diffuse>
            </material>
          </visual>

          <collision name="${prefix}_link_collision">
            <pose relative_to="${prefix}_link">0 0 0 ${pi/2} 0 0</pose>
            <geometry>
              <cylinder>
                <radius>${wheel_radius}</radius>
                <length>${wheel_width}</length>
              </cylinder>
            </geometry>
          </collision>

          <xacro:cylinder_inertia m="0.5" r="${wheel_radius}" h="${wheel_width}"/>
        </link>

        <joint name="${prefix}_joint" type="revolute">
          <parent>base_link</parent>
          <child>${prefix}_link</child>
          <pose relative_to="base_link">${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ${-wheel_zoff} 0 0 0</pose>
          <axis>
            <xyz>0 1 0</xyz>
            <limit>
              <lower>-inf</lower>
              <upper>inf</upper>
            </limit>
          </axis>
        </joint>
      </xacro:macro>

      <xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1" />
      <xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1" />

      <link name="front_caster">
        <pose relative_to="caster_joint"/>

        <visual name="front_caster_visual">
          <geometry>
            <sphere>
              <radius>${(wheel_radius+wheel_zoff-(base_height/2))}</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 1 1</ambient>
            <diffuse>0 1 1 1</diffuse>
          </material>
        </visual>

        <collision name="front_caster_collision">
          <geometry>
            <sphere>
              <radius>${(wheel_radius+wheel_zoff-(base_height/2))}</radius>
            </sphere>
          </geometry>
          <surface><friction><ode>
            <mu>0.001</mu>
            <mu2>0.001</mu2>
          </ode></friction></surface>
        </collision>

        <xacro:sphere_inertia m="0.5" r="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </link>

      <joint name="caster_joint" type="fixed">
        <parent>base_link</parent>
        <child>front_caster</child>
        <pose relative_to="base_link">${caster_xoff} 0.0 ${-(base_height/2)} 0 0 0</pose>
      </joint>

      <joint name='imu_joint' type='fixed'>
        <parent>base_link</parent>
        <child>imu_link</child>
        <pose relative_to="base_link">0.0 0.0 0.01 0 0 0</pose>
      </joint>

      <link name='imu_link'>
        <pose relative_to="imu_joint"/>
        <visual name="imu_link_visual">
          <geometry>
            <box><size>
              0.1 0.1 0.1
            </size></box>
          </geometry>
        </visual>

        <collision name="imu_link_collision">
          <geometry>
            <box><size>
              0.1 0.1 0.1
            </size></box>
          </geometry>
        </collision>

        <xacro:box_inertia m="0.1" w="0.1" d="0.1" h="0.1"/>

        <sensor name="imu_sensor" type="imu">
          <always_on>true</always_on>
          <update_rate>100</update_rate>
          <visualize>true</visualize>
          <topic>demo/imu</topic>
          <gz_frame_id>imu_link</gz_frame_id>
          <imu>
            <angular_velocity>
              <x>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>2e-4</stddev>
                  <bias_mean>0.0000075</bias_mean>
                  <bias_stddev>0.0000008</bias_stddev>
                </noise>
              </x>
              <y>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>2e-4</stddev>
                  <bias_mean>0.0000075</bias_mean>
                  <bias_stddev>0.0000008</bias_stddev>
                </noise>
              </y>
              <z>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>2e-4</stddev>
                  <bias_mean>0.0000075</bias_mean>
                  <bias_stddev>0.0000008</bias_stddev>
                </noise>
              </z>
            </angular_velocity>
            <linear_acceleration>
              <x>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>1.7e-2</stddev>
                  <bias_mean>0.1</bias_mean>
                  <bias_stddev>0.001</bias_stddev>
                </noise>
              </x>
              <y>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>1.7e-2</stddev>
                  <bias_mean>0.1</bias_mean>
                  <bias_stddev>0.001</bias_stddev>
                </noise>
              </y>
              <z>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>1.7e-2</stddev>
                  <bias_mean>0.1</bias_mean>
                  <bias_stddev>0.001</bias_stddev>
                </noise>
              </z>
            </linear_acceleration>
          </imu>
        </sensor>
      </link>

      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <!-- wheels -->
        <left_joint>drivewhl_l_joint</left_joint>
        <right_joint>drivewhl_r_joint</right_joint>

        <!-- kinematics -->
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>${wheel_radius}</wheel_radius>

        <!-- limits -->
        <max_linear_acceleration>0.1</max_linear_acceleration>

        <!-- input -->
        <topic>/demo/cmd_vel</topic>

        <!-- output -->
        <odom_topic>/demo/odom</odom_topic>
        <tf_topic>/tf</tf_topic>

        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
      </plugin>

      <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
      </plugin>

      <joint name="lidar_joint" type="fixed">
        <parent>base_link</parent>
        <child>lidar_link</child>
        <pose relative_to="base_link">0.0 0.0 0.12 0 0 0</pose>
      </joint>

      <link name='lidar_link'>
        <pose relative_to="lidar_joint"/>
        <visual name="lidar_link_visual">
          <geometry>
            <cylinder>
              <radius>0.0508</radius>
              <length>0.055</length>
            </cylinder>
          </geometry>
        </visual>

        <collision name="lidar_link_collision">
          <geometry>
            <cylinder>
              <radius>0.0508</radius>
              <length>0.055</length>
            </cylinder>
          </geometry>
        </collision>

        <xacro:cylinder_inertia m="0.125" r="0.0508" h="0.055"/>

        <sensor name="lidar" type="gpu_lidar">
          <always_on>true</always_on>
          <visualize>true</visualize>
          <update_rate>5</update_rate>
          <topic>scan</topic>
          <gz_frame_id>lidar_link</gz_frame_id>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1.000000</resolution>
                <min_angle>0.000000</min_angle>
                <max_angle>6.280000</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.120000</min>
              <max>3.5</max>
              <resolution>0.015000</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </ray>
        </sensor>
      </link>

      <joint name="camera_joint" type="fixed">
        <parent>base_link</parent>
        <child>camera_link</child>
        <pose relative_to="base_link">0.215 0 0.05 0 0 0</pose>
      </joint>

      <link name='camera_link'>
        <pose relative_to="camera_joint"/>
        <visual name="camera_link_visual">
          <geometry>
            <box><size>
              0.015 0.130 0.0222
            </size></box>
          </geometry>
        </visual>

        <collision name="camera_link_collision">
          <geometry>
            <box><size>
              0.015 0.130 0.0222
            </size></box>
          </geometry>
        </collision>

        <xacro:box_inertia m="0.035" w="0.015" d="0.130" h="0.0222"/>

        <sensor name="depth_camera" type="rgbd_camera">
          <always_on>true</always_on>
          <visualize>true</visualize>
          <update_rate>30.0</update_rate>
          <topic>depth_camera</topic>
          <gz_frame_id>camera_link</gz_frame_id>
          <camera>
            <horizontal_fov>1.047198</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>3</far>
            </clip>
          </camera>
          <baseline>0.2</baseline>
          <pointCloudCutoff>0.5</pointCloudCutoff>
          <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </sensor>
      </link>
    </model>
  </sdf>

Build and Launch
================
Change the ``default_model_path`` in ``display.launch.py`` to reference the SDF description instead of the URDF one.

.. code-block:: python

  default_model_path = os.path.join(pkg_share, 'src', 'description', 'sam_bot_description.sdf')

Now build and source your package and launch ``display.launch.py``:

.. code-block:: shell

  colcon build --symlink-install
  source install/setup.bash
  ros2 launch sam_bot_description display.launch.py

Conclusion
==========

And that's it. In this tutorial, you have successfully created a SDF for a simple differential drive robot. You have also set up a ROS 2 project that launches a robot publisher node, which then uses your SDF to publish the robot's transforms. We have also used RViz to visualize our robot to verify whether our SDF is correct.
