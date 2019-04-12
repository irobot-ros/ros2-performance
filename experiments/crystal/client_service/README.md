# Client / Service performances



## Latency

When measuring the latency of a client / service system, it is important to take into account that each iteration is made of two messages.
Moreover the content of the two messages can be very different in size (i.e. a GetMap service: the request is empty, the response is a big map).

Let's start considering the simplest possible case: the request and the response are equal and both contains just an Header message.

In case of a 1 service 1 client system, the latency is slightly more than double the one for the 1 publisher 1 subscriber case in reliable mode.

Adding more clients to the same service however the latency increases much more than in the pub/sub case.
The reason is that, being this a 1-to-1 communication, each client will receive a "different" response.

![Plot](client_vs_sub_latency.png)



## Memory [RaspberryPi 3]

The memory requirement is almost identical to the one of publishers/subscribers.

| ROS2 system | Distribution | DDS        |  Physical RAM  | Virtual RAM
| ------------- | ------------- | ------------- | ------------- | ------------- |
| minimal_client       | Crystal      | FastRTPS   | 11MB           | 87MB
| minimal_service        | Crystal      | FastRTPS   | 11MB           | 77MB



