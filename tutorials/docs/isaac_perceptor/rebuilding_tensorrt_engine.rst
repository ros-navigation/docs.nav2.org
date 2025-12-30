.. _rebuilding_tensorrt_engine:

Rebuilding TensorRT Engine for Isaac Perceptor on Nova Carter
*************************************************************

This is a step-by-step guide for fixing Isaac Perceptor model ("Engine") compatibility issues in the NVIDIA Isaac environment. While this has been developed (and tested) on the NVIDIA Nova Carter robot, this should work for Isaac Sim, etc. as well.

Among the collection of nodes and packages Perceptor uses for 3-D scene reconstruction are a set of "engine" and "plan" files, which are the actual neural network models used by Perceptor components such as ``nvblox`` to do things like object recognition, semantic segmentation, image disparity calculation, etc.

**Problem**: Incompatible "engine" files.

When running the Isaac ROS Perceptor node, the following error occurs::

  Error Code 6: API Usage Error (The engine plan file is not compatible with this version of TensorRT, expecting library version 10.7.0.23)

**Root Cause**: The .engine files are shipped pre-compiled for a generic CUDA runtime built with the specific version of TensorRT installed on the system running Perceptor. Since Perceptor is typically run from a Docker container vs. in the native host Jetpack install, there is often some version drift between the Docker image the container runs and the host Jetpack.

**Solution**: Fortunately, NVIDIA provides some fine-grained tools for working with CUDA, building & converting models between different formats and NVIDIA hardware platforms. So, we can rebuild the incompatible ``.engine`` files with the ``trtexec`` tool.

Rebuild engines inside the container using ``trtexec`` compiled against the container's TensorRT version with "full" runtime optimization.

.. note::

  We will see why "full" is important later, although sneak-peek: it has to do with resolving the error message::

    Error Code 4: API Usage Error (Cannot deserialize engine with lean runtime...

Step-by-Step Resolution
========================

This all needs to be done inside the container you will eventually be running Isaac Perceptor from.
Do all these steps from top to bottom, preferably run from inside a ``screen(1)`` session.

1. Access the Running Container
-------------------------------

.. code-block:: bash

  docker exec -it <container_name> /bin/bash

2. Rebuild ``trtexec`` for Compatibility
----------------------------------------

.. code-block:: bash

  # Navigate to TensorRT source
  cd /usr/src/tensorrt

  # Clean and rebuild trtexec
  sudo make clean
  sudo make -j$(nproc) trtexec

  # Verify new trtexec version
  ./bin/trtexec --version

3. Reinstall ISAAC_ROS Assets
-----------------------------

.. code-block:: bash

  # Reinstall essential model packages
  sudo apt-get install --reinstall isaac_ros_ess_models_install
  # Add other model packages as needed

4. Remove Old Engine Files
--------------------------

.. code-block:: bash

  # Navigate to assets directory
  cd $ISAAC_ROS_ASSETS

  # Remove all existing engine files
  find . -name "*.engine" -delete
  find . -name "*.plan" -delete

5. Modify Model Installation Scripts
------------------------------------

For each model install script (``/opt/isaac_ros_assets/install_scripts/*.sh``):

.. code-block:: bash

  # Edit the trtexec command to add --useRuntime=full
  sed -i 's/trtexec --onnx=/trtexec --useRuntime=full --onnx=/' install_script.sh

6. Regenerate Engines with Full Runtime
---------------------------------------

.. code-block:: bash

  # Run each modified installation script
  sudo /opt/isaac_ros_assets/install_scripts/isaac_ros_ess_models_install.sh
  # Repeat for other model scripts as needed

7. Commit the Container Changes
-------------------------------

.. code-block:: bash

  # From host system
  docker ps  # Get container ID/name
  docker commit <container_name> nova_carter_with_new_engines

8. Launch Perceptor
-------------------

.. code-block:: bash

  # Inside container
  ros2 launch nova_carter_bringup perceptor.launch.py

If Perceptor reads in the new .engine files successfully, you should see output similar to this on the console for each camera used::

  [component_container_mt-9] [INFO] [1765602288.918936247] [right_stereo_camera.left.rectify_node]: Negotiating
  [INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/right_stereo_camera/ess_node' in container 'nova_container'
  [component_container_mt-9] [INFO] [1765602288.919007703] [right_stereo_camera.right.rectify_node]: Negotiating
  [component_container_mt-9] [INFO] [1765602288.920391927] [nova_container]: Load Library: /opt/ros/humble/lib/libvisual_slam_node.so

As the .engine models are used by multiple nodes, such as ``dnn_stereo_disparity`` and ``nvblox``, a successful Perceptor run will result in a LOT of console output, but eventually quiet down and stay running.
