# PauseResumeController { #pause-resume-controller }

Controlled through service calls to pause and resume the execution of the tree.
It has one mandatory child for the RESUMED, and three optional for the PAUSED state, the ON_PAUSE event and the ON_RESUME event.
It has two input ports:

- pause_service_name: name of the service to pause
- resume_service_name: name of the service to resume

The controller starts in RESUMED state, and ticks it until it returns success.
When the pause service is called, ON_PAUSE is ticked until completion, then the controller switches to PAUSED state.
When the resume service is called, ON_RESUME is ticked until completion, then the controller switches back to RESUMED state.

The controller only returns success when the RESUMED child returns success.
The controller returns failure if any child returns failure.
In any other case, it returns running.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/control/pause_resume_controller.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
