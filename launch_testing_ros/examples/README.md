# Examples

## `talker_listener.test.py`

Usage:
> launch_test examples/talker_listener.test.py

This test launches the talker and listener example nodes from demo_nodes_py and interacts
with them via their ROS interfaces.  Remapping rules are used so that one of the tests can sit in
between the talker and the listener and change the data on the fly.

Node that in the setUpClass method, the test makes sure that the listener is subscribed and
republishing messages.  Since the listener process provides no synchronization mechanism to
inform the outside world that it's up and running, this step is necessary especially in resource
constrained environments where process startup may take a non negligible amount of time.  This
is often the cause of "flakyness" in tests on CI systems.  A more robust design of the talker and
listener processes might provide some positive feedback that the node is up and running, but these
are simple example nodes.

#### test_fuzzy_data
This test gives an example of what a test that fuzzes data might look like.  A ROS subscriber
and publisher pair encapsulated in a `DataRepublisher` object changes the string "Hello World" to
"Aloha World" as it travels between the talker and the listener.

#### test_listener_receives
This test publishes unique messages on the `/chatter` topic and asserts that the same messages
go to the stdout of the listener node

#### test_talker_transmits
This test subscribes to the remapped `/talker_chatter` topic and makes sure the talker node also
writes the data it's transmitting to stdout
