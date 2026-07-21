# Acoustic communication semantics

Blue Drake provides a deterministic packet-schedule abstraction for experiments
that need acoustic timing and interference without a hardware-in-the-loop modem.
It does not emulate a vendor protocol, generate packet contents, or open a
network connection.

## DiveNET Sealink profile

The only selectable modem is `divenet-sealink-3km-oem`. Product names identify
a simulation target and do not imply affiliation, endorsement, or protocol
compatibility.

The following envelope values are published on the
[DiveNET Sealink product page](https://www.divenetgps.com/sealink), retrieved
2026-07-21:

| Parameter | Represented value | Provenance |
|---|---:|---|
| Supported data rates | 78, 156, 314, 634 bit/s | Published |
| Maximum range | 3,000 m | Published product maximum |
| Depth rating | 1,000 m | Published option; hardware dependent |
| Code channels | 20 | Conservative representation of “over 20” |
| Devices per channel | 254 | Published maximum |
| Selected simulation data rate | 634 bit/s | Assumed selection from published rates |
| Maximum payload | 64 bytes | Assumed simulation bound |
| Fixed preamble time | 0.08 s | Assumed simulation overhead |

Because the payload and preamble values are assumptions, the profile remains
`provisional`. They must not be presented as Sealink wire-format limits.

## Timing model

For payload size `n`, selected data rate `R`, assumed preamble `t_p`, stationary
range `d`, and configured sound speed `c`, Blue Drake calculates:

```text
airtime = t_p + 8 n / R
propagation = d / c
one-way latency = airtime + propagation
```

The scenario runner evaluates all `[[network.transmissions]]` against vehicle
positions at the beginning of the run. Each event reports range, transmit and
arrival timing, and one of these outcomes:

- `delivered`: within the profile range with no modeled conflict,
- `out_of_range`: range exceeds the profile maximum,
- `out_of_medium`: either endpoint is above the water surface,
- `collision`: arrivals overlap at one receiver on one code channel,
- `half_duplex`: the receiver transmits while the payload arrives, or
- `transmit_conflict`: one source is asked to transmit overlapping payloads.

Code channels are treated as mutually orthogonal. Range is straight-line 3D
distance, sound speed is constant, and positions do not move during schedule
evaluation. Event ordering is deterministic and does not use wall time or
random state.

Endpoint phase currently uses each vehicle body origin as the modem-location
proxy. An origin at `z = 0` is considered wet so a surface vehicle may carry
the acoustic gateway. Explicit modem mount offsets and time-varying immersion
are deferred fidelity work.

## TOML example

```toml
[network]
modem = "divenet-sealink-3km-oem"
sound_speed_mps = 1500.0

[[network.transmissions]]
id = "surface-to-rov"
source = "surface_1"
destination = "rov_1"
start_time_s = 0.1
payload_bytes = 16
code_channel = 0
```

IDs must be unique; vehicle references, payload bounds, code channels, and
scenario timing are validated when TOML is loaded.

## Deliberate omissions

- DiveNET subscriber addressing, pseudo-NMEA messages, framing, and firmware,
- bit errors, packet-error curves, capture effect, coding, and retransmission,
- frequency-dependent attenuation, ambient noise, multipath, and Doppler,
- propagation through bathymetry, boundaries, or sound-speed profiles,
- moving-node geometry during a scheduled payload,
- clocks, queues, medium-access control, routing, and application messages, and
- sockets, serial ports, operational endpoints, credentials, ROS, HIL, or C2.

Popoto remains intentionally deferred. A future modem profile must document its
own sources and cannot inherit Sealink assumptions silently.
