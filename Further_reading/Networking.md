<h1 align="center"> Understanding ROS 2 Networking and Communication </h1>

<h2 align="center"> Context </h2>

ROS 2 and Gazebo (other simulators are available) both rely on network-based middleware to allow distributed components (nodes, processes) to automatically discover each other and exchange data.
Even when everything runs on a single machine, all communication still flows through the operating system’s network stack, using User Datagram Protocol (UDP) and multicast, exactly as it would between devices on a local area network (LAN).

ROS 2 uses the same fundamental networking concepts found in large-scale distributed computing: IP addressing, UDP ports, multicast discovery, and firewall rules.
Understanding these layers makes it far easier to design system archetecture and diagnose the “nodes not found”, “no topics visible”, or “Gazebo black GUI” problems that frequently appear in real deployments.

---

<h2 align="center"> Rationale </h2>

According to the [ROS 2 Design Rationale](https://design.ros2.org/articles/ros_on_dds.html), ROS 2 was deliberately built on top of the Data Distribution Service (DDS) standard, a mature, open middleware used widely in aerospace, automotive, and other distributed systems.

The reasoning was:
* To replace ROS 1’s custom TCPROS/UDPROS transports with an industrial-grade communication layer capable of real-time performance, redundancy, and security.
* To inherit the benefits of DDS, including peer-to-peer discovery, configurable Quality of Service (QoS) policies, and flexible transport options.
* To leverage existing, well-tested network infrastructure rather than reinventing message passing from scratch.

In practice, this means every ROS 2 process is a DDS “participant” that communicates through standard Internet Protocol (IP) networking.
DDS handles the discovery of peers via UDP multicast, then switches to unicast UDP for data exchange — the same principle that Gazebo’s own Gazebo Transport system uses for synchronising simulation state and GUI data.

This network-centric design gives ROS 2 great flexibility, nodes can run anywhere: on the same host, across a local network, or over a Virtual Private Network (VPN).
However, it also means that standard network constraints still apply: firewall rules, Docker network isolation, multicast routing, or blocked UDP ports can all disrupt discovery and communication.

Understanding these fundamentals is therefore essential not only for debugging and system integration, but also for designing robust robotic systems that behave consistently in both simulation and real hardware environments.
---
<h2 align="center"> The Network Stack </h2>

To understand how ROS 2 (and Gazebo) communicate, it helps to look at how data actually moves through a computer. Communicationd ride on top of the TCP/IP networking stack, see table 1.

Network communication on Linux follows the TCP/IP stack, see table 1. Each layer in this stack adds its own responsibilities, and problems at any layer can result in communication disruption.
<div align="center">

| Layer               | Example protocols & components   | Role in communication                                                    | Typical ROS 2/Gazebo relevance                                   |
| ------------------- | -------------------------------- | ------------------------------------------------------------------------ | ---------------------------------------------------------------- |
| **Application**     | DDS, Gazebo Transport, HTTP, SSH | Defines what messages mean   (sensor data, transforms, simulation state) | ROS 2 topics, services, actions                                  |
| **Middleware**      | DDS                              | Manages peer discovery, Quality of Service, and pub/sub semantics        | Handles discovery, QoS, and message delivery policies            |
| **Transport**       | UDP, TCP                         | Moves data between programs on different devices                         | ROS 2 and Gazebo primarily use UDP for low-latency messaging     |
| **Network**         | IP (IPv4/IPv6)                   | Provides logical addressing and routing between hosts                    | IP addresses for nodes on the same machine or network            |
| **Link / Physical** | Ethernet, Wi-Fi, virtual bridges | Moves packets across physical or virtual hardware                        | Determines actual connectivity (wired, wireless, Docker bridge)  |

</div>
<p align="center">
Table 1. Modified TCP/IP Stack Summary.
</p>

Even if both nodes are on the same computer, the packets still travel through this stack internally via the loopback interface (`lo`), obeying the same routing and firewall rules as any external traffic.
This layered model explains why ROS 2 and Gazebo can experience network-type issues — such as blocked multicast, unreachable peers, or port conflicts, even in “local” simulations.

**Let's map this to ROS 2**
When applying the TCP/IP stack to ROS 2 or any real-life senario the layers are less distinct and can merge together.


```
User code (ROS 2 Node)            ← Application logic
        │
ROS 2 Client Library (rclcpp/rclpy) ← ROS 2 abstraction layer
        │
Data Distribution Service (DDS)   ← Middleware over UDP/IP
        │
UDP / IP Stack                    ← Transport & Network layers
        │
Ethernet / Wi-Fi / Loopback       ← Link & Physical layers
```
The user code defines the behaviour and logic of your robot or simulation (publishers, subscribers, services). The ROS 2 client library (e.g. `rclcpp` or `rclpy`) translates those high-level actions into DDS calls. DDS then handles discovery, matching, and Quality of Service, sending data over UDP. The UDP/IP stack ensures packets reach the correct host and port. Finally, the network interface (wired, wireless, or virtual) delivers the packets physically — even if that’s just to the loopback adapter when running locally. This perspective ties your node’s `publish()` and `subscribe()` calls directly to the actual network mechanisms that make ROS 2 communication work.

---

<h2 align="center"> IP Addresses and Ports </h2>

Every packet sent through the network stack must know where it’s going and who it’s for. In computer networking, those two pieces of information are defined by the IP address and the port number.

**IP Addressess - Where**
