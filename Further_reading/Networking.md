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

| Layer               | Example protocols & components   | Role in communication                                                    |  ROS 2/Gazebo relevance                                          |
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
An IP address identifies a specific device, network interface, or logical group on a network, see table 2. IP addresses provides the location of the device within the network and tells the routers and switches where to deliver packets (data).

<div align="center">
        
| Type            | Typical range                 | Purpose                                    | ROS 2/Gazebo relevance                   |
| --------------- | ----------------------------- | ------------------------------------------ | ---------------------------------------- |
| **Loopback**    | `127.0.0.1`                   | The local machine only                     | Used when running nodes on a single host |
| **Private LAN** | `192.168.x.x`, `10.x.x.x`     | Local networks not visible on the internet | Common in lab or robot networks          |
| **Public**      | e.g. `8.8.8.8`                | Routable on the internet                   | Rarely used for robotics systems         |
| **Multicast**   | `224.0.0.0 – 239.255.255.255` | One-to-many communication groups           | Used heavily by DDS and Gazebo Transport |
</div>
<p align="center">
Table 2. IPv4 Summary.
</p>
Multicast addresses are special because they represent a group of devices that have joined that address, not a single host.
ROS 2 and Gazebo both depend on multicast groups for automatic discovery:
* ROS 2 DDS: `239.255.0.1`
* Gazebo Transport: `239.255.0.7`
When a node or Gazebo server starts, it joins the appropriate multicast group so other participants on the same network can detect it.

**Ports - Who**
A port number identifies a specific service or process on that IP address.
Each device can support up to 65,535 ports (0–65535), and the operating system uses them to deliver packets to the correct application.

<div align="center">
        
| Range           | Name               | Typical use                    | Example                                     |
| --------------- | ------------------ | ------------------------------ | ------------------------------------------- |
| **0–1023**      | *Well-known ports* | Reserved for system services   | 22 (SSH), 80 (HTTP)                         |
| **1024–49151**  | *Registered ports* | Applications and middleware    | 7400–7500 (ROS 2 DDS), 10317–10318 (Gazebo) |
| **49152–65535** | *Ephemeral ports*  | Temporary outbound connections | Used dynamically by the OS                  |
</div>
<p align="center">
Table 3. Port allocation.
</p>
> [!TIP]
> If you're trying to remote access a device using SSH and it fails or hangs. Checking SSH is enabled and that port 22 isn't blocked are key troubleshooting steps.
> As you are troubleshooting you may find yourself using commands such as `ros2 doctor --report` and `ros2 doctor --network`.

Together, the IP address and port number form a socket, a unique endpoint for communication.
For example:
`239.255.0.1:7400`-`239.255.0.1:7500`   → ROS 2 DDS discovery multicast
`239.255.0.7:10317`  → Gazebo discovery multicast (outbound)
`239.255.0.7:10318`  → Gazebo discovery listener (inbound)

Even if multiple processes share the same IP, their distinct ports prevent collisions, each DDS participant or Gazebo instance binds its own sockets.

You can see this yourself by opening a terminal sourcing ROS and running the talker demo node
```
ros2 run demo_nodes_cpp talker
```
While this is running, open another terminal and run
```
sudo tcpdump -n -i any udp port 7400
```
The terminal output will show the packets.
Additioanlly, you could run gazebo and run
```
netstat -g | grep 239
```
`netstat -g` lists all multicast group memberships for each network interface on your system. `grep 239` filters that list to show only IPv4 multicast addresses in the 239.x.x.x range.  Expect entries for `239.255.0.1` ROS 2 and `239.255.0.7` if Gazebo is running.

If you have Gazebo running and want to see packets in and outbound then run:
```
sudo tcpdump -n -i any udp port 10317 or udp port 10318
```

---

<h2 align="center"> Transport protocol and Multicast </h2>

**Transport protocol**

Once a process knows which IP and port to use, it still needs a transport protocol. A transport protocol is a set of rules for how packets are sent and received.
In ROS 2 and Gazebo, that protocol is usually User Datagram Protocol (UDP),  because it’s fast, lightweight, and ideal for real-time robotic data. In a robotics setting timing usually matters more than guaranteed delivery.
An alternative to UDP is Transmission Control Protocol (TCP), TCP requires a handshake which confirms the recipt of packets which impacts latency. UDP sends packets and then moves on, it's well suited to high-frequency, low-latency communication such as sensor streams, control loops, and simulation updates.
Gazebo and ROS 2 each use UDP differently:

* ROS 2 DDS: Uses UDP both for discovery and data transmission.
* Gazebo Transport: Uses UDP multicast for discovery, then either UDP or TCP for data, depending on the message type.

**Mulitcasing**

Multicast is a special feature of the IP layer that allows a single sender to deliver packets to multiple receivers simultaneously.
Instead of broadcasting to everyone (which is inefficient), multicast targets only those systems that have explicitly “joined” a multicast group.
In short, multicast is how Gazebo and DDS participants find each other automatically, without configuration.

**DDS Discovery**

DDS discovery is the process that uses multicast (and unicast) to find and connect those peers.
Each ROS 2 process (DDS participant) joins the multicast group, i.e. `239.255.0.1:7400`. It sends a Simple Participant Discovery Protocol (SPDP) packet: *“I’m here; these are my endpoints.”*. Other participants receive it, add the sender to their discovery database, and respond via unicast UDP. Once matched, all further topic data flows directly over unicast connections.
> [!NOTE]
> In relation to DDS, you will come across the term Real-Time Publish-Subscribe (RTPS) protocol. RTPS is the network protocol that implements DDS communication.

**Gazebo Transport**

Gazebo Transport’s design is almost identical to DDS, see Table 3.

<div align="center">
| Role                    | Address                 | Ports       | Protocol   | Description                        |
| ----------------------- | ----------------------- | ----------- | ---------- | ---------------------------------- |
| **Discovery (send)**    | 239.255.0.7             | 10317       | UDP        | Server and GUI announce themselves |
| **Discovery (receive)** | 239.255.0.7             | 10318       | UDP        | Listens for peers                  |
| **Data exchange**       | Unicast (dynamic ports) | OS-assigned | UDP or TCP | Actual simulation messages         |

</div>
<p align="center">
Table 4. Gazebo Transport Summary.
</p>

> [!TIP]
> If the multicast packets on ports 10317/10318 are blocked by a firewall or by Docker’s virtual bridge, the GUI never sees the server which causes the “black screen” symptom.

**Troubleshooting**
You can inspect in action multicast behaviour directly by:
```
# See which multicast groups your interfaces have joined
netstat -g
ip maddr show

# Capture multicast packets in real time
sudo tcpdump -n -i any udp and multicast

# Test DDS multicast manually
ros2 multicast send &
ros2 multicast receive
```
Common causes of multicast failure in ROS 2 and Gazebo environment are outlined in Table 4.
<div align="center">
| Cause                         | Where it occurs    | Effect                            |
| ----------------------------- | ------------------ | --------------------------------- |
| Host firewall (UFW, nftables) | Operating system   | Blocks UDP multicast packets      |
| Docker / container bridge     | Virtual network    | Drops multicast by default        |
| Loopback multicast disabled   | Kernel parameter   | Prevents local self-discovery     |
| Network ACLs / campus routers | LAN infrastructure | Multicast blocked between subnets |

</div>
<p align="center">
Table 5. Common networking issues Summary.
</p>

A common cause for networking issues is that UDP multicast or discovery ports are blocked by the host firewall.
You can address this by explicitly opening the required UDP ports and multicast range using the **u**ncomplicated **f**ire**w**all (UFW).
```
# ROS 2 (DDS/RTPS) discovery + unicast data control ports
# Default DDS (FastDDS) uses UDP ports 7400–7500 for discovery and data
sudo ufw allow 7400:7500/udp

# Gazebo Transport discovery (GUI ↔ server)
# Uses UDP ports 10317–10318 for multicast discovery messages
sudo ufw allow 10317:10318/udp

# Allow secure remote access via SSH (TCP 22)
sudo ufw allow ssh

# Permit local-site multicast groups
# DDS and Gazebo both use 239.255.0.x addresses for discovery
sudo ufw allow to 239.255.0.0/16 proto udp

# ──────────────────────────────────────────────────────────────
# Enable firewall (no-operation if already active) and verify rules
sudo ufw enable
sudo ufw status verbose
```
If you ever want to remove this config run:
```
sudo ufw delete allow 7400:7500/udp
sudo ufw delete allow 10317:10318/udp
sudo ufw delete allow to 239.255.0.0/16 proto udp
sudo ufw delete allow ssh
```
