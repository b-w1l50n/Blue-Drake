# Contributing to Blue Drake

Blue Drake follows the engineering spirit of Drake's contribution process while
remaining an independent downstream project.

The acceptance boundary for the current release is maintained in the
[1.0 release contract](docs/release_contract.md), and compatibility commitments
are listed in the [public API policy](docs/public_api.md).

## Before coding

For a change spanning multiple pull requests, open a design issue first. State:

- the physical behavior and intended users,
- governing equations and references,
- frames, units, inputs, state, outputs, and parameters,
- fidelity limitations and validation evidence,
- scalar-conversion expectations, and
- a sequence of reviewable changes.

Prefer pull requests below 750 changed lines. Pull requests over 1,500 changed
lines require an explicit maintainer exception.

## Required quality

Every model change must include:

- deterministic tests,
- at least one physical invariant or reference benchmark,
- documentation of assumptions and omitted effects,
- no hidden wall-clock or global random state,
- no network, filesystem, or GUI dependency in core dynamics, and
- formatting and lint compliance.

Run the local checks with:

```bash
ruff check .
ruff format --check .
pytest
blue-drake benchmark
python -m build
python -m twine check dist/*
```

Core systems should use Drake `Context` state and parameters. External
protocols, visualization, and hardware adapters belong at application
boundaries. Vendor names must be accompanied by parameter provenance and a
non-endorsement statement.

Scenario schema changes require an explicit versioning decision, strict parser
tests, and updated JSON inspection examples. Run artifacts are append-free:
tools must create a new directory and must not overwrite prior experiment data.
Generic planners must remain independent of controllers and mission state.
Compatibility changes must update `docs/compatibility.md`. A release must build
an sdist and wheel, validate both with Twine, install the wheel into a clean
environment, and smoke-test version, scenario validation, and benchmarks.

## Public boundary

Do not contribute Erinyes autonomy, mission behavior, operational endpoints,
credentials, HIL orchestration, private datasets, or learned-model services.
Generic controllers and path-planning algorithms are acceptable when they are
independently documented and tested.
