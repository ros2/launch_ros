# launch\_testing\_ros

## Examples

### `talker_listener_launch_test.py`

Usage:

```sh
launch_test test/examples/talker_listener_launch_test.py
```

This test launches the talker and listener example nodes from demo\_nodes\_py and interacts
with them via their ROS interfaces.  Remapping rules are used so that one of the tests can sit in
between the talker and the listener and change the data on the fly.

#### test\_fuzzy\_data
This test gives an example of what a test that fuzzes data might look like.  A ROS subscriber
and publisher pair encapsulated in a `DataRepublisher` object changes the string "Hello World" to
"Aloha World" as it travels between the talker and the listener.

#### test\_listener\_receives
This test publishes unique messages on the `/chatter` topic and asserts that the same messages
go to the stdout of the listener node

#### test\_talker\_transmits
This test subscribes to the remapped `/talker_chatter` topic and makes sure the talker node also
writes the data it's transmitting to stdout
