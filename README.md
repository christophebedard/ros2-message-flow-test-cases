# ROS 2 message flow test cases

This repo contains simple test cases for different message link types needed to extract the flow of messages in ROS 2.
See the [ROS 2 message flow analysis repository](https://github.com/christophebedard/ros2-message-flow-analysis).

This is part of the [ROS 2 message flow paper](https://arxiv.org/abs/2204.10208).
If you use or refer to this method or this repository, please cite:
* C. Bédard, P.-Y. Lajoie, G. Beltrame, and M. Dagenais, "Message Flow Analysis with Complex Causal Links for Distributed ROS 2 Systems," *Robotics and Autonomous Systems*, vol. 161, p. 104361, 2023.

BibTeX:

```bibtex
@article{bedard2023messageflow,
  title={Message flow analysis with complex causal links for distributed {ROS} 2 systems},
  author={B{\'e}dard, Christophe and Lajoie, Pierre-Yves and Beltrame, Giovanni and Dagenais, Michel},
  journal={Robotics and Autonomous Systems},
  year={2023},
  volume={161},
  pages={104361},
  doi={10.1016/j.robot.2022.104361}
}
```

## Message link types

1. [Synchronous 1-to-N](./src/sync_one_to_n.cpp)
    * New messages are published on N topics directly in the subscription callback for a received message
    * No instrumentation or annotation necessary
1. [Periodic asynchronous N-to-M](./src/periodic_async_n_to_m.cpp)
    * Messages are received from N topics and are cached
    * A timer periodically uses the last messages from each topic to compute a result and publish messages on M topics
        * In case the timer callback is executed when at least one topic has not received a message yet (i.e., empty cache), no new message(s) are published
        * This behaviour is defined for the sake of simplicity and could be adapted or changed
    * **Message link annotation is required**
    * Equivalent to the [cyclic node](https://github.com/ros-realtime/reference-system/blob/main/reference_system/include/reference_system/nodes/rclcpp/cyclic.hpp), but with M output topics instead of 1
1. [Partially synchronous N-to-M](./src/partial_sync_n_to_m.cpp)
    * Messages are received from N topics and are cached
    * Every time a new message is received and put into the corresponding cache, all topic caches are checked in the callback itself
        * If (at least) one new message for each input topic has been received, a result is computed and messages are published on M topics in the subscription callback itself
        * Then all the cache is reset and at least one new message for each topic needs to be received for the new message(s) to be published
    * **Message link annotation is required**
    * Equivalent to the [fusion node](https://github.com/ros-realtime/reference-system/blob/main/reference_system/include/reference_system/nodes/rclcpp/fusion.hpp), but with N input topics and M output topics instead of 2 and 1, respectively

## Base nodes

1. [Source](./src/source.cpp)
    * Publishes messages on N topics periodically using a timer
1. [Sink](./src/sink.cpp)
    * Receives messages from N topics
