# Groot - Interacting with Behavior Trees

## Visualize Behavior Trees

To display a Behavior Tree like that in [Figure 3](#groot-nav2-default-bt), we will first start the Groot executable.
Out of the box, Groot can only display Behavior Trees and nodes that are from the defaults in BT.CPP, since it does not know anything about Nav2 or your other projects.
Therefore, we must point Groot to our palette, or index of Nav2 / custom behavior tree nodes:

1. Open Groot in editor mode. Now, Groot should look like in [Figure 1](#groot-bt-editor).
2. Select the Load palette from file option either via the context menu or the import icon in the top middle of the menu bar.
3. Open the file /path/to/navigation2/nav2_behavior_tree/nav2_tree_nodes.xml to import all the custom behavior tree nodes used for navigation. This is the palette of Nav2 custom behavior tree nodes. Now, Groot should look like in [Figure 2](#groot-bt-editor-with-nodes).
4. Select Load tree option near the top left corner
5. Browse the tree you want to visualize, then select OK. The Nav2 BTs exist in /path/to/navigation2/nav2_bt_navigator/behavior_trees/

<div markdown="span" style="display: flex; gap: 10px;">
    <figure markdown="span" id="groot-bt-editor">
        ![](images/Groot/groot_bt_editor.png){ width="400px" }
        <figcaption>Figur 1: Default Editor View</figcaption>
    </figure>
    <figure markdown="span" id="groot-bt-editor-with-nodes">
        ![](images/Groot/groot_with_nav2_custom_nodes.png){ width="397px" }
        <figcaption>Figure 2: Editor with Custom Nodes loaded in blue</figcaption>
    </figure>
</div>

If you select the default tree navigate_w_replanning_and_recovery.xml, then a Groot editor should look like [Figure 3](#groot-nav2-default-bt).

<figure markdown="span" id="groot-nav2-default-bt">
    ![](images/Groot/bt_w_replanning_and_recovery.png)
    <figcaption>Figure 3: Full Nav2 Default BehaviorTree</figcaption>
</figure>

!!! note

    If a tree cannot be visualized because some nodes are missing in the palette, you might need to add it to your palette. While we try to keep Nav2’s BT nodes and palettes in sync, if you notice one is missing, please file a ticket or pull request and we should have that updated quickly.

## Edit Behavior Trees

Now that you have a Nav2 BT open in Groot in editor mode, you should be able to trivially modify it using the GUI.
Starting from a screen like that shown in [Figure 3](#groot-nav2-default-bt), you can pull in new nodes from the side panel to add them to the workspace.
You may then connect the nodes using a “drag and drop” motion between the node’s input and output ports to assemble the new nodes into the tree.

If you select a given node, you can change metadata about it such as its name or values of parameterizable ports. When you’re done modifying, simply save the new configuration file and use that on your robot the next time!

## Adding A Custom Node

Each node in the behavior tree holds a specialized function.
Sometimes, its useful to create new nodes and add them to your palette during the design process - perhaps before the implementations themselves exist.
This helps designers abstract away the implementation specifics of the nodes from the higher level logic of the tree itself and how they’d like to interact with a given node (e.g. type, ports, etc).
Within Groot, you may create new custom nodes to add to your tree and export these new nodes back to your palette.
Implementing the node itself needs to be done separately from Groot, which is described in [Writing a New Behavior Tree Plugin](../../plugin_tutorials/docs/writing_new_bt_plugin.md#writing-a-new-behavior-tree-plugin).

<figure markdown="span" id="groot-create-custom-node">
    ![](images/Groot/groot_create_custom_node.png)
    <figcaption>Figure 4: Create a new Custom Node</figcaption>
</figure>

Creating a new custom node can be started by clicking the orange marked icon in [Figure 4](#groot-create-custom-node), while Groot is in Editor mode.
This should load a new window, similar to [Figure 5](#groot-interactive-node-creation).
In this new window, it asks you to fill in the metadata about this new node, in order to create it.
It will ask you for standard information such as name (green box), type of node (orange box), and any optional ports for parameterization or access to blackboard variables (blue box).

After completing, select OK in [Figure 5](#groot-interactive-node-creation), the new custom node should appear in blue in the TreeNode Palette as in [Figure 6](#groot-export-new-node).

<div markdown="span" style="display: flex; gap: 10px; height: 350px">
    <figure markdown="span" id="groot-interactive-node-creation" style="object-fit: fill;">
        ![](images/Groot/groot_interactive_node_creation.png){ width="350px" }
        <figcaption>Figure 5: UI to describing new Nodes</figcaption>
    </figure>
    <figure markdown="span" id="groot-export-new-node" style="object-fit: fill;">
        ![](images/Groot/groot_export_new_node.png){ width="342px" }
        <figcaption>Figure 6: Exporting the new Custom Node</figcaption>
    </figure>
</div>

Before starting to create a new BT based on the new custom nodes, it is recommend to export the newly created nodes to save in case of Groot crashing.
This can be performed with the icon highlighted in green from [Figure 6](#groot-export-new-node).
The resulting XML output from the node created in [Figure 5](#groot-interactive-node-creation) can be seen below.
You can see more examples in [Nav2’s BT Node Palette XML](https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/nav2_tree_nodes.xml).

```xml
<root>
    <TreeNodesModel>
        <Action ID="MyAwesomeNewNode">
            <input_port name="key_name" default="false">coffee</input_port>
            <output_port name="key_name2" default="42">Sense of life</output_port>
            <inout_port name="next_target" default="pancakes">rolling target</inout_port>
        </Action>
    </TreeNodesModel>
</root>
```
