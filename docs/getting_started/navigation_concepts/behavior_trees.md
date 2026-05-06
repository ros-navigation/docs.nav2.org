# Behavior Trees { #behavior-trees }

Behavior trees (BT) are becoming increasingly common in complex robotics tasks.
They are a tree structure of tasks to be completed.
It creates a more scalable and human-understandable framework for defining multi-step or many state applications.
This is opposed to a finite state machine (FSM) which may have dozens of states and hundreds of transitions.
An example would be a soccer-playing robot.
Embedding the logic of soccer game play into a FSM would be challenging and error prone with many possible states and rules.
Additionally, modeling choices like to shoot at the goal from the left, right, or center, is particularly unclear.
With a BT, basic primitives, like "kick", "walk", "go to ball", can be created and reused for many behaviors.
More information can be found [in this book](https://arxiv.org/abs/1709.00084).
I **strongly** recommend reading chapters 1-3 to get a good understanding of the nomenclature and workflow.
It should only take about 30 minutes.

Behavior Trees provide a formal structure for navigation logic which can be both used to create complex systems but also be verifiable and validated as provenly correct using advanced tools. Having the application logic centralized in the behavior tree and with independent task servers (which only communicate data over the tree) allows for formal analysis.

For this project, we use [BehaviorTree CPP V4](https://www.behaviortree.dev/) as the behavior tree library.
We create node plugins which can be constructed into a tree, inside the `BT Navigator`.
The node plugins are loaded into the BT and when the XML file of the tree is parsed, the registered names are associated.
At this point, we can march through the behavior tree to navigate.

One reason this library is used is its ability to load subtrees. This means that the Nav2 behavior tree can be loaded into another higher-level BT to use this project as node plugin.
An example would be in soccer play, using the Nav2 behavior tree as the "go to ball" node with a ball detection as part of a larger task.
Additionally, we supply a `NavigateToPoseAction` plugin (among others) for BT so the Nav2 stack can be called from a client application through the usual action interface.

Other systems could be used to design complex autonomous behavior, namely Hierarchical FSMs (HFSM).
Behavior Trees were selected due to popularity across the robotics and related industries and by largely user demand.
However, due to the independent task server nature of Nav2, it is not difficult to offer a `nav2_hfsm_navigator` package in the future, pending interest and contribution.
