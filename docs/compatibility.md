# Release compatibility and version policy

## Supported release configuration

Blue Drake 0.1 supports CPython 3.12 through 3.14. Its CI release gate runs all
three versions on Ubuntu 24.04 with Drake 1.54.0. This matches Drake's currently
documented Ubuntu 24.04 pip configurations:

- [Drake supported configurations](https://drake.mit.edu/installation.html)
- [Drake 1.54.0 on PyPI](https://pypi.org/project/drake/1.54.0/)

Install the complete supported simulation environment with:

```bash
python3 -m pip install '.[drake-current]'
```

The core dependency set contains NumPy but not Drake. That permits lightweight
scenario validation, inspection, catalogs, planning, acoustic scheduling, and
analytical benchmarks without downloading Drake. Simulation and diagram APIs
require one of the explicit Drake extras.

## macOS and the legacy Sonoma host

Current supported macOS releases should use `drake-current`. They are not yet a
Blue Drake CI release gate. Drake ended macOS 14 Sonoma support after 1.45.0, so
the original Sonoma development host uses the deliberately frozen legacy extra:

```bash
python3 -m pip install '.[drake-sonoma]'
```

That combination is retained for local development only and receives no
upstream fixes. See Drake's
[end-of-support notice](https://drake.mit.edu/release_notes/end_of_support.html)
and [1.46.0 release notes](https://drake.mit.edu/release_notes/v1.46.0.html).

## Package and API versions

Blue Drake uses semantic package versions. While the package major version is
zero, public Python APIs may change between minor releases; breaking changes
must be called out in the changelog and should use a deprecation period when
practical.

Three data contracts are versioned independently:

- TOML scenarios use `schema_version` (currently 1),
- run manifests use `artifact_schema_version` (currently 1), and
- benchmark JSON uses `benchmark_schema_version` (currently 1).

Unknown scenario versions are rejected. A breaking contract change requires a
new schema version, strict parser and serialization tests, documentation, and a
migration example. Adding optional fields with stable defaults may remain within
the existing schema when old files retain identical meaning.

Drake systems in 0.1 are double-only. AutoDiff and symbolic scalar conversion
are not supported or implied.
