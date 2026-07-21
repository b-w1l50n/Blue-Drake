from __future__ import annotations

import json
from pathlib import Path

from blue_drake.inspection import catalog_summary, scenario_summary
from blue_drake.scenario import load_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_scenario_summary_is_json_ready_and_preserves_units() -> None:
    summary = scenario_summary(
        load_scenario(REPO_ROOT / "scenarios" / "mixed_marine.toml")
    )
    assert summary["schema_version"] == 1
    assert summary["environment"]["seafloor_z_W_m"] == -20.0
    assert summary["environment"]["air_density_kg_m3"] == 1.225
    assert summary["environment"]["air_temperature_C"] == 18.0
    assert {vehicle["kind"] for vehicle in summary["vehicles"]} == {
        "glider",
        "rov",
        "usv",
        "uuv",
    }
    assert summary["network"]["transmission_count"] == 4
    surface = summary["vehicles"][0]
    assert surface["wrench_command_B"] == [0.0, 0.0, 0.0, 8.0, 0.0, 0.0]
    assert surface["wind_velocity_W_mps"] == [2.0, 0.0, 0.0]
    assert surface["sensors"][0]["bias"] == [0.0] * 6
    assert [event["status"] for event in summary["network"]["events"]] == [
        "delivered",
        "delivered",
        "collision",
        "collision",
    ]
    json.dumps(summary)


def test_catalog_contains_all_current_builtin_families() -> None:
    catalog = catalog_summary()
    assert {item["preset"] for item in catalog["vehicles"]} == {
        "glider",
        "rov",
        "usv",
        "uuv",
    }
    assert {item["profile_id"] for item in catalog["sensors"]} >= {
        "blue-robotics-bar30",
        "xsens-avior-ahrs",
    }
    assert [item["profile_id"] for item in catalog["modems"]] == [
        "divenet-sealink-3km-oem"
    ]
    json.dumps(catalog)
