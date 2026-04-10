# ConcatenatePaths { #concatenate-paths }

Concatenates two paths into a single path, in order such that the output is `input_path1 + input_path2`.
May be used with multiple of these calls sequentially to concatenate multiple paths.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ConcatenatePaths input_path1="{main_path}" input_path2="{last_mile_path}" output_path="{path}"/>
```
