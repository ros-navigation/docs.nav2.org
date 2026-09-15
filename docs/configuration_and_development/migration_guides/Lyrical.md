# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## New MPPI Axis Align Critic

A new critic, `AxisAlignCritic`, was added to the MPPI controller for holonomic platforms.
It penalizes diagonal motion (commanding `vx` and `vy` at the same time), which on mecanum bases only drives two of the four wheels and slips on real hardware, while leaving pure forward/backward and pure lateral motion free.
It is inactive for non-holonomic motion models and is not part of the default critic set; add `AxisAlignCritic` to `critics` to enable it.
See the [MPPI configuration guide](../configuration_guide/controller_plugins/mppi_controller/configuring_mppic.md#axis-align-critic) for parameters.
