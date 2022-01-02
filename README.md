# ROS 2 message flow test cases

This repo contains simple test cases for different message link types needed to figure out the flow of messages in ROS 2.

## Message link types

1. 1-to-N
    * New messages are published on N topics directly in the subscription callback for a received message
    * No instrumentation or annotation necessary
1. N-to-M
    * Messages are received from N topics and are cached
    * A timer periodically uses the last messages from each topic to compute a result and publish messages on M topics
    * Annotation is required
1. 2-to-N
    * Messages are received from 2 topics and are cached
    * Once at least one new message for each topic has been received, a result is computed and messages are published on N topics
    * Annotation is required
    * Note: could be M-to-N
