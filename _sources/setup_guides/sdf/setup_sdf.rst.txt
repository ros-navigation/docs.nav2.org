.. _sdf_handson:

Setting Up The SDF - Gazebo
###########################

For this guide, we will be creating the SDF (Simulation Description Format) file for a simple differential drive robot to give you hands-on experience on building an SDF file for Gazebo simulation.

.. seealso::
  The complete source code in this tutorial can be found in `navigation2_tutorials <https://github.com/ros-navigation/navigation2_tutorials/tree/master/sam_bot_description>`_ repository under the ``sam_bot_description`` package. Note that the repository contains the full code after accomplishing all the tutorials in this guide.

About SDF
=========

SDF is a file format for simulators, like Gazebo, that describes the simulator environment, models (including its links, connections, and physics simulator metadata), and appropriate plugins. The SDF that we will make is for modern Gazebo, but there are other simulator options such as Open3D Engine or Isaac Sim.

We can also use our SDF with the robot_state_publisher using the following package. You will see how to do this in the tutorial below.

.. code-block:: shell

  sudo apt install ros-<ros2-distro>-sdformat-urdf

This package contains a C++ library and urdf_parser_plugin for converting SDFormat XML into URDF C++ structures. Installing it allows one to use SDFormat XML instead of URDF XML as a robot description.

.. seealso::
  If you want to learn more about the SDF and sdformat_urdf, we encourage you to have a look at the official `SDFormat Website <http://sdformat.org/>`__ and the `sdformat_urdf GitHub repository <https://github.com/ros/sdformat_urdf/tree/rolling/sdformat_urdf>`__

Writing the SDF
================

For now the SDF will pretty much be a copy of the URDF code converted into SDFormat. Changes between the two descriptions will happen when we start adding various plugins and sensors in the next tutorials. We also add in physical properties needed by the simulator such as the ineria, mass, and material.

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
        </collision>

        <xacro:sphere_inertia m="0.5" r="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </link>

      <joint name="caster_joint" type="fixed">
        <parent>base_link</parent>
        <child>front_caster</child>
        <pose relative_to="base_link">${caster_xoff} 0.0 ${-(base_height/2)} 0 0 0</pose>
      </joint>
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

.. image:: ../urdf/images/base-bot_3.png

Conclusion
==========

And that's it. In this tutorial, you have successfully created a SDF for a simple differential drive robot. You have also set up a ROS 2 project that launches a robot publisher node, which then uses your SDF to publish the robot's transforms. We have also used RViz to visualize our robot to verify whether our SDF is correct.
