# 5 DOF ROBOT ARM

## Overview
This is a hobby project where I designed and built a custom 5 DOF 3D printed robot arm, as well as a graphical user interface (GUI) to control the robot arm. The purpose is to explore communication protocols, robot kinematics, and learn design patterns (among many other concepts).  

## Hardware
The robot uses Feetech STS3215 (30kg) serial bus servo motors, which provide more control and feedback than traditional hobby servo motors which often only allow position control (no feedback at all). The serial bus servo motors can be daisy-chained using cables with Molex 5264 connectors, which reduce the amount of wiring clutter. The low-level control of the motors is programmed on an ESP32 module using Feetech's Arduino libraries.  

High-level robot controls are hosted on a Raspberry Pi 4B, while the GUI was developed on Ubuntu in an x86 virtual machine.

## Software  
The Raspberry Pi 4B hosts the following components:
- Socket server
- Message handler
- Protocol parser
- Robot class (high level control library)
- Kinematics class (computation library)
  
The GUI application is written using the Qt framework and designed using Qt Creator. A 3rd part library, breeze, was used for styling.

## High Level Project Architecture - Binary Protocol
I wrote a binary protocol to manage the communication between the Python server and the C++ client via TCP connection. The server and client communicate using streams of byte arrays that contain various pieces of information such as the type of message being sent (`CONNECT`, `HOME`, `DISABLE`, `MOVE_X`, etc.) and the payload (such as Cartesian coordinates or joint angles). They share a common set of definitions that they both understand, which makes this system language-agnostic.

### Global Definitions
- The message type values mirror each other on both the client and server side.
- Both sides expect a byte array "frame" of length 64.

### Robot/Server Side (Python)
- A socket server listens for incoming messages. When messages are received, the socket server continues to accumulate bytes until the buffer is full (64 bytes).
- The message is passed to a `MessageHandler` class, which calls a `decode_message` method from the `ProtocolParser` class to extract the message type and payload, then the `MessageHandler` class calls the `message_handler` method that corresponds to the message type that was received.
- To send messages (such as joint angle feedback and message responses) to the client, an `encode_message` method in the `ProtocolParser` class is used to convert the payload into a byte array

### GUI/Client side (C++)
- The client uses the same class architecture as the server.
- A `MessageHandler` class is used to route messages based on the message type.
- A `ProtocolParser` for encoding/decoding messages.
- The Qt application uses a signal/slot mechanism to relate user interactions with functions. For example, sending a `HOME` command in the client will also emit a signal to read joint angles, and the GUI will react to this signal by populating/updating the joint angle values displayed in spinboxes.

