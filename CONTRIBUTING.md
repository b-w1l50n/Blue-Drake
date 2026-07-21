# Contributing to Blue Drake

Blue Drake follows the engineering spirit of Drake's contribution process while
remaining an independent downstream project.

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
```

Core systems should use Drake `Context` state and parameters. External
protocols, visualization, and hardware adapters belong at application
boundaries. Vendor names must be accompanied by parameter provenance and a
non-endorsement statement.

## Public boundary

Do not contribute Erinyes autonomy, mission behavior, operational endpoints,
credentials, HIL orchestration, private datasets, or learned-model services.
Generic controllers and path-planning algorithms are acceptable when they are
independently documented and tested.

