# Transform Synchronization Strategy { #transform-synchronization-strategy }

Nav2 utilizes and produces data in many frames of reference with respect to the vehicle and the environment. Many of its sources are asynchronous and are made available for use at different times. As such, it is important to have a consistent strategy for how to manage internal computations, frames of reference, and sensor data to ensure proper synchronization.

The following is a description of where to use what kinds of the Nav2 Transform Manager APIs and why.

## Choosing the lookup time-stamp

| Data or operation | Timestamp to use | Transform manager API |
| --- | --- | --- |
| Sensor observations, such as scans and point clouds | The observation's capture timestamp | `getStampedTransform(target_frame, source_frame, stamp, transform_tolerance)` |
| Stamped computed results, such as paths and poses | The result's own timestamp | `transformStampedPose(target_frame, pose, transform_tolerance)` for poses; `getStampedTransform()`  for other data |
| Operations driven by their own timing that fetch the robot's current pose or another latest frame | The latest common time available along the TF chain | `getLatestPose()` or `getLatestTransform()`, with a positive staleness threshold when old TF must be rejected |
| A source observation at one time expressed in a target frame at another time | Both timestamps, through a fixed frame such as `odom` | `getStampedTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform_tolerance)` |

When a current robot pose or transform is needed in a non-data driven context to provide a reference timestamp, obtain the latest pose or transform once and carry its timestamp through all downstream calculations so that they all reference the same instance.

Otherwise, preserve the original timestamp of sensor data and computed results unless an operation intentionally changes the time being represented.

Use `nav2::TransformManager` from `nav2_ros_common/tf_manager.hpp` for these lookups and for `sendTransform()` or `sendStaticTransform()` when broadcasting.
