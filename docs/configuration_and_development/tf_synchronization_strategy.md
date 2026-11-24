# Transform Synchronization Strategy { #transform-synchronization-strategy }

Nav2 utilizes and produces data in many frames of reference with respect to the vehicle and the environment. Many of its sources are asynchronous and are made available for use at different times. As such, it is important to have a consistent strategy for how to manage internal computations, frames of reference, and sensor data to ensure proper synchronization.

Tf operations are facilitated by the `nav2::TransformManager`, which provides a shared interface for receiving, looking up, and broadcasting transforms throughout Nav2. It reduces repeated TF2 boilerplate and makes transform handling consistent across components.

## Choosing the lookup timestamp and Transform Manager API

You can refer to the following guide to choose which combination of timestamp and API provided by the transform manager better fits your use case.

| What you need | Timestamp to use | Transform Manager API |
| --- | --- | --- |
| Transform sensor observations, such as scans and point clouds, or stamped computed results other than poses | The observation's capture timestamp or the result's own timestamp | `getStampedTransform(target_frame, source_frame, stamp, transform_tolerance)` |
| Express a stamped pose in another frame | The input pose's timestamp | `transformStampedPose(target_frame, input_pose, transform_tolerance)` |
| Get the robot's or another frame's current pose for a control loop or periodic check, with no incoming data timestamp | The latest common time available along the TF chain | `getLatestPose(target_frame, source_frame, staleness_threshold)` |
| Get the current transform between frames for an operation with no incoming data timestamp | The latest common time available along the TF chain | `getLatestTransform(target_frame, source_frame, staleness_threshold)` |
| Express an observation from one time in a target frame at another time, accounting for motion | Both timestamps, through a fixed frame such as `odom` | `getStampedTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform_tolerance)` |

### Waiting and timestamp handling

While choosing an API, account for how it handles unavailable transforms:

- **Latest** lookups return immediately. A positive `staleness_threshold` rejects outdated dynamic transforms. See [Transform staleness checking](#transform-staleness-checking) below for configuration details.
- **Stamped** lookups can wait up to `transform_tolerance` (a `double` in seconds) for the required TF. This is a wait timeout, not an allowance for timestamp error or extrapolation.

### Handling lookup results

Lookup methods return `std::optional`; check the result before using it. An optional `error_msg` receives diagnostic details on failures.

### Timestamps propagation

Subsequent tf operations on related data should reuse coherent time-stamps. When the operation has no input data timestamp, obtain the latest pose/transform once, then reuse its timestamp downstream; otherwise preserve the data’s original timestamp. Avoid using now() as reference for transformations lookups unless you have a motivated reason: this can can request a transform newer than the available TF and potentially result in unnecessary wait time or extrapolation errors.

## Broadcasting and other utilities

Use `sendTransform(transform)` or `sendStaticTransform(transform)` to publish dynamic or static transforms.

If you only need to broadcast, pass `activate_listener=false` when constructing the manager to avoid unnecessary overhead.

The static methods `transformToPoseStamped()`, `poseToTransformStamped()`, and `invertTransform()` convert between pose and transform messages or invert a transform; they do not look up TF.

## Native TF integrations

If your application requires access to the native implementation of `tf2_ros::Buffer`, you can still access it through `getRawBuffer()`, which exposes it as a shared pointer, but the manager API should be preferred whenever possible.

Pass the manager to `nav2::create_message_filter()` for TF-aware subscriptions.

## Transform staleness checking

When a dynamic TF publisher stops updating, its last transform can still be returned as the latest available one. Navigation based on that old pose can produce incorrect commands or miss hazards. Staleness checking lets a component reject an outdated latest transform and report the failure so the robot can respond safely.

The `transform_staleness_threshold` parameter sets the maximum allowed age, in seconds, of transforms retrieved using the latest available timestamp. If set to 0.0, it disables the age check. Static-only TF chains are exempt. Choose a threshold that covers the normal update period and expected latency of your dynamic transforms, but still catches a stopped publisher promptly.

Set `transform_staleness_threshold` in each relevant component's configuration when you want it to reject stale TF; it is not a single stack-wide setting.
