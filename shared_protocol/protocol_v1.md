\# SO-101 TCP Bridge Protocol v1



\## 1. Purpose



This protocol connects two isolated local environments:



\- ROS2 Lyrical client

\- LeRobot hardware server



The connection uses localhost TCP only.



The protocol does not directly expose ROS2 messages or LeRobot Python objects.

All transmitted data must use protocol-defined JSON messages.



\## 2. Transport



\- Address: 127.0.0.1

\- Port: 8765

\- Protocol: TCP over IPv4

\- Encoding: UTF-8

\- Framing: one JSON object per line

\- Line terminator: \\n

\- Maximum message size: 65536 bytes

\- Only one ROS2 client is allowed in protocol v1



The server must bind only to 127.0.0.1 and must not bind to 0.0.0.0.



\## 3. Common message envelope



Every message must contain:



\- protocol\_version

\- type

\- seq

\- timestamp\_monotonic

\- payload



Example:



```json

{

&#x20; "protocol\_version": "1.0",

&#x20; "type": "heartbeat",

&#x20; "seq": 4,

&#x20; "timestamp\_monotonic": 12345.67,

&#x20; "payload": {}

}



protocol\_version



Protocol version used to reject incompatible clients and servers.



Protocol v1 uses:



1.0

type



Identifies how the receiver should interpret payload.



Allowed protocol v1 message types:



hello

hello\_ack

heartbeat

joint\_state

joint\_command

command\_ack

estop

clear\_estop

error

seq



An integer sequence number.



Each sender maintains its own sequence counter.



The first outgoing message uses seq=1.

The counter increases by one for every outgoing message.

A new TCP connection starts a new sequence.

Receivers may reject duplicate or stale sequence numbers.

timestamp\_monotonic



The sender's local monotonic timestamp in seconds.



It is used for logging and diagnostics.



It must not be used to calculate cross-process timeout directly.

Timeout uses the receiver's local time when a valid message arrives.



payload



An object containing data specific to the message type.



4\. Connection state machine



ROS2 client state:



DISCONNECTED

&#x20;   -> CONNECTING

&#x20;   -> HANDSHAKING

&#x20;   -> ACTIVE



Failure transition:



ACTIVE

&#x20;   -> STALE

&#x20;   -> DISCONNECTED

&#x20;   -> CONNECTING



The client sends hello immediately after TCP connection.



The server must not publish joint states or accept commands before the

handshake has completed successfully.



5\. Handshake

hello



Sent by the ROS2 client immediately after connection.



Required payload fields:



role

session\_id

supported\_protocol\_versions

capabilities

hello\_ack



Sent by the LeRobot server after validating hello.



Required payload fields:



accepted\_version

server\_mode

hardware\_connected

command\_enabled

joint\_names



During stage 0:



server\_mode must be simulation

hardware\_connected must be false

command\_enabled must be false



If no common protocol version exists, the server sends error and closes the

connection.



6\. Heartbeat



Both peers send heartbeat every 0.2 seconds after successful handshake.



A peer is considered stale if no valid message of any type has been received

for 1.0 second.



Receiving any valid protocol message refreshes the local peer timeout timer.



Required heartbeat payload fields:



state

last\_rx\_seq

hardware\_connected

command\_enabled

estop\_active



On heartbeat timeout:



ROS2 client:



marks the connection STALE

stops publishing fresh joint states

publishes a disconnected diagnostic state

closes the socket

attempts reconnection after 1.0 second



LeRobot server:



rejects new commands

discards queued or expired commands

must not continue executing stale commands

closes the connection

waits for a new client



Stage 0 uses simulated data and performs no physical action.



7\. Joint state



joint\_state transfers robot observations from the LeRobot server to ROS2.



Required payload fields:



names

position



Optional fields:



velocity

effort



Rules:



names and position must have the same length

velocity and effort, if present, must have the same length as names

position unit is radians

velocity unit is radians per second

effort is not used in stage 0

joint names in stage 0 must be explicitly marked as mock names



The final real joint names will be read from the actual LeRobot SO-101

configuration. They must not be guessed or hard-coded before hardware

inspection.



8\. Joint command



joint\_command is reserved for future trajectory execution.



Required payload fields:



command\_id

mode

names

position

duration\_s



Protocol v1 initially supports only:



mode=position



During stage 0 command\_enabled=false, therefore every joint\_command must be

rejected and must never be forwarded to hardware.



9\. Command acknowledgement



command\_ack reports whether a joint command was accepted.



Required payload fields:



command\_id

accepted

reason

server\_state



During stage 0:



accepted=false

reason=command\_disabled\_in\_stage\_0

10\. Emergency stop



estop is a high-priority, idempotent message.



Required payload field:



reason



Once active, emergency stop is latched.



Normal commands must not automatically clear emergency stop.



clear\_estop is a separate request and will be implemented only after a safe

state machine is defined.



11\. Error message



Required payload fields:



code

message

fatal

related\_seq



A fatal error causes the connection to close after the error message is sent.



12\. Safety rules

The server binds only to 127.0.0.1.

Hardware connection is disabled by default.

Command execution is disabled by default.

Unknown message types are rejected.

Invalid JSON is rejected.

Messages larger than 65536 bytes are rejected.

Incompatible protocol versions are rejected.

Commands before handshake are rejected.

Commands after timeout are rejected.

Duplicate or stale commands may not be executed.

Stage 0 does not open serial ports.

Stage 0 does not instantiate a real LeRobot robot.

Stage 0 does not send any motor command.

