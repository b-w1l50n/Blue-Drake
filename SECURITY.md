# Security policy

Blue Drake 0.1 is an offline simulation toolkit, not a vehicle-control or
operational communications system. Do not use it as a safety boundary and do
not place credentials, private endpoints, mission data, or proprietary logs in
scenarios, examples, tests, or bug reports.

The project does not implement network clients, vendor protocols, ROS nodes,
HIL interfaces, C2 workflows, or credential storage. Meshcat is supplied by
Drake and listens on `localhost` by default. Passing `--meshcat-host '*'`
exposes its unauthenticated HTTP and WebSocket listener on every interface.
Use that option only on a trusted, firewalled LAN; never publish the port to the
internet.

For a suspected vulnerability, use the repository host's private vulnerability
reporting feature when available or contact the maintainers privately. Do not
publish secrets or exploit details in a public issue. Include the affected Blue
Drake version, platform, dependency versions, minimal reproduction, and impact.

Only the current 0.1 release line is intended to receive security fixes. Drake,
NumPy, and other dependencies follow their own support and disclosure policies.
