# PipelineSequence { #pipeline-sequence }

Ticks the first child till it succeeds, then ticks the first and second children till the second one succeeds.
It then ticks the first, second, and third children until the third succeeds, and so on, and so on. If at any
time a child returns RUNNING, that doesn't change the behavior. If at any time a child returns FAILURE, that
stops all children and returns FAILURE overall.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/control/pipeline_sequence.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
