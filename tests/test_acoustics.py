from __future__ import annotations

import pytest

from blue_drake.acoustics import (
    DIVENET_SEALINK_3KM_OEM,
    AcousticDeliveryStatus,
    AcousticTransmissionRequest,
    estimate_transmission,
    modem_profile,
    schedule_transmissions,
)


def test_divenet_is_the_only_initial_selectable_modem() -> None:
    profile = modem_profile("divenet-sealink-3km-oem")
    assert profile is DIVENET_SEALINK_3KM_OEM
    assert profile.validation_status == "provisional"
    assert profile.supported_data_rates_bps == (78.0, 156.0, 314.0, 634.0)
    assert profile.code_channel_count == 20
    assert profile.maximum_devices_per_channel == 254
    assert profile.source_url == "https://www.divenetgps.com/sealink"
    with pytest.raises(ValueError, match="unknown acoustic modem"):
        modem_profile("popoto")


def test_ideal_timing_includes_airtime_and_sound_propagation() -> None:
    estimate = estimate_transmission(
        DIVENET_SEALINK_3KM_OEM,
        payload_bytes=64,
        range_m=1500.0,
    )
    assert estimate.in_range
    assert estimate.propagation_s == pytest.approx(1.0)
    assert estimate.airtime_s == pytest.approx(0.88757, abs=1e-5)
    assert estimate.latency_s == pytest.approx(1.88757, abs=1e-5)


def test_range_and_payload_limits_are_explicit() -> None:
    estimate = estimate_transmission(
        DIVENET_SEALINK_3KM_OEM,
        payload_bytes=8,
        range_m=3000.1,
    )
    assert not estimate.in_range
    with pytest.raises(ValueError, match="payload_bytes"):
        DIVENET_SEALINK_3KM_OEM.airtime_s(65)


def _request(
    transmission_id: str,
    source: str,
    destination: str,
    *,
    start_time_s: float = 0.0,
    code_channel: int = 0,
) -> AcousticTransmissionRequest:
    return AcousticTransmissionRequest(
        transmission_id=transmission_id,
        source_id=source,
        destination_id=destination,
        start_time_s=start_time_s,
        payload_bytes=16,
        code_channel=code_channel,
    )


def test_stationary_schedule_delivers_nonoverlapping_transmissions() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={"a": (0, 0, 0), "b": (1500, 0, 0)},
        requests=(_request("one", "a", "b"),),
    )
    assert events[0].status is AcousticDeliveryStatus.DELIVERED
    assert events[0].range_m == pytest.approx(1500.0)
    assert events[0].one_way_latency_s == pytest.approx(
        DIVENET_SEALINK_3KM_OEM.airtime_s(16) + 1.0
    )


def test_overlapping_receptions_on_same_code_channel_collide() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={
            "a": (-10, 0, 0),
            "b": (10, 0, 0),
            "receiver": (0, 0, 0),
        },
        requests=(
            _request("a_to_r", "a", "receiver"),
            _request("b_to_r", "b", "receiver"),
        ),
    )
    assert {event.status for event in events} == {
        AcousticDeliveryStatus.COLLISION
    }


def test_three_overlapping_receptions_all_collide() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={
            "a": (-10, 0, 0),
            "b": (10, 0, 0),
            "c": (0, 10, 0),
            "receiver": (0, 0, 0),
        },
        requests=(
            _request("a_to_r", "a", "receiver"),
            _request("b_to_r", "b", "receiver"),
            _request("c_to_r", "c", "receiver"),
        ),
    )
    assert all(
        event.status is AcousticDeliveryStatus.COLLISION for event in events
    )


def test_overlapping_transmissions_from_one_source_report_conflict() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={
            "source": (0, 0, 0),
            "a": (10, 0, 0),
            "b": (-10, 0, 0),
        },
        requests=(
            _request("to_a", "source", "a"),
            _request("to_b", "source", "b"),
        ),
    )
    assert all(
        event.status is AcousticDeliveryStatus.TRANSMIT_CONFLICT
        for event in events
    )


def test_separate_code_channels_do_not_collide() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={
            "a": (-10, 0, 0),
            "b": (10, 0, 0),
            "receiver": (0, 0, 0),
        },
        requests=(
            _request("a_to_r", "a", "receiver", code_channel=0),
            _request("b_to_r", "b", "receiver", code_channel=1),
        ),
    )
    assert all(
        event.status is AcousticDeliveryStatus.DELIVERED for event in events
    )


def test_receiver_transmission_causes_half_duplex_failure() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={"a": (0, 0, 0), "b": (10, 0, 0)},
        requests=(
            _request("a_to_b", "a", "b"),
            _request("b_to_a", "b", "a", start_time_s=0.1),
        ),
    )
    assert all(
        event.status is AcousticDeliveryStatus.HALF_DUPLEX for event in events
    )


def test_out_of_range_event_is_not_delivered() -> None:
    event = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={"a": (0, 0, 0), "b": (3001, 0, 0)},
        requests=(_request("far", "a", "b"),),
    )[0]
    assert event.status is AcousticDeliveryStatus.OUT_OF_RANGE


def test_acoustic_event_is_out_of_medium_when_endpoint_is_in_air() -> None:
    events = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={"air": (0, 0, 0.01), "water": (10, 0, -1)},
        requests=(
            _request("air_to_water", "air", "water"),
            _request("water_to_air", "water", "air", start_time_s=1.0),
        ),
    )
    assert all(
        event.status is AcousticDeliveryStatus.OUT_OF_MEDIUM for event in events
    )


def test_acoustic_event_is_out_of_depth_below_modem_rating() -> None:
    event = schedule_transmissions(
        DIVENET_SEALINK_3KM_OEM,
        node_positions_W_m={
            "deep": (0, 0, -1000.01),
            "water": (10, 0, -1),
        },
        requests=(_request("deep_to_water", "deep", "water"),),
    )[0]
    assert event.status is AcousticDeliveryStatus.OUT_OF_DEPTH
