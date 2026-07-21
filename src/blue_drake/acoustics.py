"""Hardware-informed acoustic modem and propagation configuration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class ModemProfile:
    """Parameters required by the initial packet timing model.

    ``validation_status`` prevents provisional values from being mistaken for
    manufacturer-certified performance.
    """

    profile_id: str
    display_name: str
    protocol_family: str
    supported_data_rates_bps: tuple[float, ...]
    maximum_payload_bytes: int
    data_rate_bps: float
    maximum_range_m: float
    depth_rating_m: float
    code_channel_count: int
    maximum_devices_per_channel: int
    preamble_s: float
    validation_status: str
    source_url: str
    source_retrieved: str = "2026-07-21"

    def __post_init__(self) -> None:
        if not self.profile_id.strip() or not self.display_name.strip():
            raise ValueError("modem profile identifiers cannot be empty")
        if self.maximum_payload_bytes <= 0:
            raise ValueError("maximum_payload_bytes must be positive")
        rates = tuple(float(rate) for rate in self.supported_data_rates_bps)
        if not rates or any(
            rate <= 0.0 or not math.isfinite(rate) for rate in rates
        ):
            raise ValueError(
                "supported_data_rates_bps must be positive and finite"
            )
        object.__setattr__(self, "supported_data_rates_bps", rates)
        if self.data_rate_bps not in rates:
            raise ValueError(
                "data_rate_bps must be one of supported_data_rates_bps"
            )
        for name in ("data_rate_bps", "maximum_range_m", "depth_rating_m"):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")
        if self.code_channel_count <= 0:
            raise ValueError("code_channel_count must be positive")
        if self.maximum_devices_per_channel <= 0:
            raise ValueError("maximum_devices_per_channel must be positive")
        if self.preamble_s < 0.0 or not math.isfinite(self.preamble_s):
            raise ValueError("preamble_s must be finite and nonnegative")
        if self.validation_status not in {"provisional", "validated"}:
            raise ValueError("unsupported validation_status")
        if not self.source_url.startswith("https://"):
            raise ValueError("source_url must use https")

    def airtime_s(self, payload_bytes: int) -> float:
        """Return preamble plus uncoded payload serialization time."""

        if not 0 < payload_bytes <= self.maximum_payload_bytes:
            raise ValueError(
                "payload_bytes must be in [1, maximum_payload_bytes]"
            )
        return self.preamble_s + 8.0 * payload_bytes / self.data_rate_bps


DIVENET_SEALINK_3KM_OEM = ModemProfile(
    profile_id="divenet-sealink-3km-oem",
    display_name="DiveNET Sealink 3 km OEM",
    protocol_family="divenet-sealink",
    supported_data_rates_bps=(78.0, 156.0, 314.0, 634.0),
    maximum_payload_bytes=64,
    data_rate_bps=634.0,
    maximum_range_m=3000.0,
    depth_rating_m=1000.0,
    code_channel_count=20,
    maximum_devices_per_channel=254,
    preamble_s=0.08,
    validation_status="provisional",
    source_url="https://www.divenetgps.com/sealink",
)


MODEM_PROFILES = {
    DIVENET_SEALINK_3KM_OEM.profile_id: DIVENET_SEALINK_3KM_OEM,
}


def modem_profile(profile_id: str) -> ModemProfile:
    """Resolve a supported acoustic modem profile."""

    try:
        return MODEM_PROFILES[profile_id]
    except KeyError as exc:
        raise ValueError(
            f"unknown acoustic modem profile: {profile_id}"
        ) from exc


@dataclass(frozen=True)
class AcousticChannelConfig:
    """Environmental parameters for initial acoustic timing checks."""

    sound_speed_mps: float = 1500.0

    def __post_init__(self) -> None:
        if self.sound_speed_mps <= 0.0 or not math.isfinite(
            self.sound_speed_mps
        ):
            raise ValueError("sound_speed_mps must be positive and finite")


@dataclass(frozen=True)
class AcousticTransmissionEstimate:
    """Deterministic airtime and propagation result without packet loss."""

    in_range: bool
    range_m: float
    airtime_s: float
    propagation_s: float

    @property
    def latency_s(self) -> float:
        """Return serialization plus one-way propagation delay."""

        return self.airtime_s + self.propagation_s


class AcousticDeliveryStatus(StrEnum):
    """Outcome of the deterministic shared-channel abstraction."""

    DELIVERED = "delivered"
    OUT_OF_RANGE = "out_of_range"
    OUT_OF_MEDIUM = "out_of_medium"
    COLLISION = "collision"
    HALF_DUPLEX = "half_duplex"
    TRANSMIT_CONFLICT = "transmit_conflict"


@dataclass(frozen=True)
class AcousticTransmissionRequest:
    """One payload scheduled on a generic acoustic code channel."""

    transmission_id: str
    source_id: str
    destination_id: str
    start_time_s: float
    payload_bytes: int
    code_channel: int = 0

    def __post_init__(self) -> None:
        for name in ("transmission_id", "source_id", "destination_id"):
            if not getattr(self, name).strip():
                raise ValueError(f"{name} cannot be empty")
        if self.source_id == self.destination_id:
            raise ValueError("acoustic source and destination must differ")
        if self.start_time_s < 0.0 or not math.isfinite(self.start_time_s):
            raise ValueError("start_time_s must be finite and nonnegative")
        if self.payload_bytes <= 0:
            raise ValueError("payload_bytes must be positive")
        if self.code_channel < 0:
            raise ValueError("code_channel must be nonnegative")


@dataclass(frozen=True)
class AcousticTransmissionEvent:
    """Calculated timing and outcome for one requested transmission."""

    request: AcousticTransmissionRequest
    status: AcousticDeliveryStatus
    range_m: float
    transmit_end_s: float
    arrival_start_s: float
    arrival_end_s: float

    @property
    def one_way_latency_s(self) -> float:
        """Return time from transmission start through final received bit."""

        return self.arrival_end_s - self.request.start_time_s


def _position(name: str, value) -> Vector3:
    try:
        position = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"position for {name} must contain three numbers"
        ) from exc
    if len(position) != 3 or not all(math.isfinite(item) for item in position):
        raise ValueError(
            f"position for {name} must contain three finite numbers"
        )
    return position


def _overlaps(
    start_a: float, end_a: float, start_b: float, end_b: float
) -> bool:
    return max(start_a, start_b) < min(end_a, end_b)


def schedule_transmissions(
    profile: ModemProfile,
    *,
    node_positions_W_m,
    requests,
    channel: AcousticChannelConfig | None = None,
) -> tuple[AcousticTransmissionEvent, ...]:
    """Schedule payload timing with range, collision, and half-duplex checks.

    Positions are stationary for the schedule. Code channels are treated as
    mutually orthogonal; this is a generic abstraction, not an implementation
    of DiveNET's subscriber code-division protocol.
    """

    if channel is None:
        channel = AcousticChannelConfig()
    positions = {
        node_id: _position(node_id, position)
        for node_id, position in dict(node_positions_W_m).items()
    }
    requests = tuple(requests)
    transmission_ids = [request.transmission_id for request in requests]
    if len(transmission_ids) != len(set(transmission_ids)):
        raise ValueError("transmission IDs must be unique")

    events = []
    for request in requests:
        if (
            request.source_id not in positions
            or request.destination_id not in positions
        ):
            raise ValueError(
                f"transmission {request.transmission_id} references an "
                "unknown node"
            )
        if request.payload_bytes > profile.maximum_payload_bytes:
            raise ValueError(
                f"transmission {request.transmission_id} exceeds maximum "
                "payload"
            )
        if request.code_channel >= profile.code_channel_count:
            raise ValueError(
                f"transmission {request.transmission_id} uses unsupported "
                "code channel"
            )
        source = positions[request.source_id]
        destination = positions[request.destination_id]
        range_m = math.dist(source, destination)
        estimate = estimate_transmission(
            profile,
            payload_bytes=request.payload_bytes,
            range_m=range_m,
            channel=channel,
        )
        transmit_end_s = request.start_time_s + estimate.airtime_s
        arrival_start_s = request.start_time_s + estimate.propagation_s
        arrival_end_s = transmit_end_s + estimate.propagation_s
        if source[2] > 0.0 or destination[2] > 0.0:
            status = AcousticDeliveryStatus.OUT_OF_MEDIUM
        elif estimate.in_range:
            status = AcousticDeliveryStatus.DELIVERED
        else:
            status = AcousticDeliveryStatus.OUT_OF_RANGE
        events.append(
            AcousticTransmissionEvent(
                request=request,
                status=status,
                range_m=range_m,
                transmit_end_s=transmit_end_s,
                arrival_start_s=arrival_start_s,
                arrival_end_s=arrival_end_s,
            )
        )

    statuses = [event.status for event in events]
    for first in range(len(events)):
        for second in range(first + 1, len(events)):
            first_event = events[first]
            second_event = events[second]
            if (
                statuses[first] is not AcousticDeliveryStatus.OUT_OF_MEDIUM
                and statuses[second] is not AcousticDeliveryStatus.OUT_OF_MEDIUM
                and first_event.request.source_id
                == second_event.request.source_id
                and _overlaps(
                    first_event.request.start_time_s,
                    first_event.transmit_end_s,
                    second_event.request.start_time_s,
                    second_event.transmit_end_s,
                )
            ):
                statuses[first] = AcousticDeliveryStatus.TRANSMIT_CONFLICT
                statuses[second] = AcousticDeliveryStatus.TRANSMIT_CONFLICT

    for index, event in enumerate(events):
        if statuses[index] is not AcousticDeliveryStatus.DELIVERED:
            continue
        for other in events:
            if other.request.source_id != event.request.destination_id:
                continue
            if _overlaps(
                event.arrival_start_s,
                event.arrival_end_s,
                other.request.start_time_s,
                other.transmit_end_s,
            ):
                statuses[index] = AcousticDeliveryStatus.HALF_DUPLEX
                break

    for first in range(len(events)):
        collision_eligible = {
            AcousticDeliveryStatus.DELIVERED,
            AcousticDeliveryStatus.COLLISION,
        }
        if statuses[first] not in collision_eligible:
            continue
        for second in range(first + 1, len(events)):
            if statuses[second] not in collision_eligible:
                continue
            first_event = events[first]
            second_event = events[second]
            same_receiver = (
                first_event.request.destination_id
                == second_event.request.destination_id
            )
            same_channel = (
                first_event.request.code_channel
                == second_event.request.code_channel
            )
            arrival_overlap = _overlaps(
                first_event.arrival_start_s,
                first_event.arrival_end_s,
                second_event.arrival_start_s,
                second_event.arrival_end_s,
            )
            if same_receiver and same_channel and arrival_overlap:
                statuses[first] = AcousticDeliveryStatus.COLLISION
                statuses[second] = AcousticDeliveryStatus.COLLISION

    return tuple(
        AcousticTransmissionEvent(
            request=event.request,
            status=status,
            range_m=event.range_m,
            transmit_end_s=event.transmit_end_s,
            arrival_start_s=event.arrival_start_s,
            arrival_end_s=event.arrival_end_s,
        )
        for event, status in zip(events, statuses, strict=True)
    )


def estimate_transmission(
    profile: ModemProfile,
    *,
    payload_bytes: int,
    range_m: float,
    channel: AcousticChannelConfig | None = None,
) -> AcousticTransmissionEstimate:
    """Estimate ideal one-way packet timing for a stationary link."""

    if channel is None:
        channel = AcousticChannelConfig()
    if range_m < 0.0 or not math.isfinite(range_m):
        raise ValueError("range_m must be finite and nonnegative")
    return AcousticTransmissionEstimate(
        in_range=range_m <= profile.maximum_range_m,
        range_m=range_m,
        airtime_s=profile.airtime_s(payload_bytes),
        propagation_s=range_m / channel.sound_speed_mps,
    )
