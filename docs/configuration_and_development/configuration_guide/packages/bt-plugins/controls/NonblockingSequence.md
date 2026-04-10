# NonblockingSequence { #nonblocking-sequence }

Ticks all child nodes until they all return SUCCESS. If any of the child nodes return RUNNING, it will continue to tick the subsequent nodes. This node will once again tick through all the child nodes if it is ticked itself. If at any time a child returns FAILURE, that stops all children and returns FAILURE overall.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<NonblockingSequence>
    <!--Add tree components here-->
</NonblockingSequence>
```
