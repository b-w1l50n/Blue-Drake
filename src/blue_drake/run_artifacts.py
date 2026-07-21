"""Safe deterministic export of in-memory Blue Drake simulation logs."""

from __future__ import annotations

import csv
import json
import math
import platform
import re
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import TYPE_CHECKING

from blue_drake._version import __version__
from blue_drake.inspection import scenario_summary
from blue_drake.scenario import MarineScenario

if TYPE_CHECKING:
    from blue_drake.simulation import MarineFleetModel

RUN_ARTIFACT_SCHEMA_VERSION = 2


def _distribution_version(distribution: str) -> str:
    try:
        return version(distribution)
    except PackageNotFoundError:
        return "unknown"


def write_run_artifacts(
    output_dir: str | Path,
    *,
    model: MarineFleetModel,
    context,
    scenario: MarineScenario,
    simulated_duration_s: float,
    logging_period_s: float,
) -> Path:
    """Create a new artifact directory containing a manifest and CSV logs.

    The target directory must not already exist. This makes accidental
    overwrites impossible and leaves run-directory lifecycle to the caller.
    """

    if not model.logs:
        raise ValueError("model does not contain enabled logs")
    for name, value in (
        ("simulated_duration_s", simulated_duration_s),
        ("logging_period_s", logging_period_s),
    ):
        if value <= 0.0 or not math.isfinite(value):
            raise ValueError(f"{name} must be positive and finite")
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    log_entries = []
    for fleet_log in model.logs:
        data = fleet_log.sink.FindLog(context)
        sample_times = data.sample_times()
        samples = data.data()
        if samples.shape[0] != len(fleet_log.columns):
            raise RuntimeError(
                f"log {fleet_log.log_id} column contract does not match data"
            )
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", fleet_log.log_id) is None:
            raise RuntimeError(f"unsafe log identifier: {fleet_log.log_id}")
        filename = f"{fleet_log.log_id}.csv"
        with (destination / filename).open(
            "x", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(("time_s", *fleet_log.columns))
            for index, time_s in enumerate(sample_times):
                writer.writerow(
                    (
                        format(float(time_s), ".17g"),
                        *(
                            format(float(value), ".17g")
                            for value in samples[:, index]
                        ),
                    )
                )
        log_entries.append(
            {
                "log_id": fleet_log.log_id,
                "file": filename,
                "columns": ["time_s", *fleet_log.columns],
                "sample_count": int(len(sample_times)),
            }
        )
    manifest = {
        "artifact_schema_version": RUN_ARTIFACT_SCHEMA_VERSION,
        "software": {
            "blue_drake_version": __version__,
            "drake_version": _distribution_version("drake"),
            "numpy_version": _distribution_version("numpy"),
            "python_version": platform.python_version(),
        },
        "simulated_duration_s": simulated_duration_s,
        "logging_period_s": logging_period_s,
        "scenario": scenario_summary(scenario),
        "logs": log_entries,
    }
    with (destination / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination
