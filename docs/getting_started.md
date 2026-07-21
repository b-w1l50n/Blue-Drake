# Getting started on Ubuntu, WSL2, and a Linux NUC

Blue Drake's release environment is CPython 3.12 through 3.14 on Ubuntu 24.04
with Drake 1.54. The same Linux workflow covers a typical headless NUC and
Ubuntu 24.04 under WSL2.

## Clean installation

Install Git and Python's virtual-environment support using the package manager
for the machine. On Ubuntu 24.04:

```bash
sudo apt update
sudo apt install git python3-venv
git clone https://github.com/b-w1l50n/Blue-Drake.git
cd Blue-Drake
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[drake-current]'
.venv/bin/blue-drake doctor
```

`doctor` is read-only. `READY` means the required Python, NumPy, and PyDrake
imports are available. A release-platform warning means the simulator may work
but that exact combination is outside the public CI matrix.

Run a short headless check before opening a browser:

```bash
.venv/bin/blue-drake validate scenarios/mixed_marine.toml
.venv/bin/blue-drake benchmark
.venv/bin/blue-drake run scenarios/mixed_marine.toml \
  --no-visualizer --duration 0.1 --realtime-rate 0
```

Then launch the five-minute showcase:

```bash
.venv/bin/blue-drake scenarios/fleet_showcase.toml
```

## WSL2

Keep the repository inside the Linux filesystem, such as
`~/Blue-Drake`, rather than under `/mnt/c`; simulation and package operations
are usually faster there. Current WSL2 installations normally forward a Linux
localhost port to the Windows browser, so open the Meshcat URL printed by Blue
Drake in Windows.

If localhost forwarding is disabled, first confirm Windows and WSL networking
rather than immediately binding Meshcat to every interface. The
`--meshcat-host '*'` option is an explicit unauthenticated LAN exposure and is
not the default workaround for general networking problems.

## Headless NUC

The safest remote workflow keeps Meshcat on the NUC loopback interface and
uses an SSH tunnel. On the NUC:

```bash
.venv/bin/blue-drake scenarios/fleet_showcase.toml \
  --meshcat-host localhost --meshcat-port 7000
```

On the laptop:

```bash
ssh -L 7000:localhost:7000 user@NUC_IP
```

Open `http://localhost:7000` on the laptop. The tunnel provides transport and
access control through SSH; Meshcat itself has no Blue Drake authentication.

On a trusted, firewalled laboratory LAN, direct access is also possible:

```bash
.venv/bin/blue-drake scenarios/fleet_showcase.toml \
  --meshcat-host '*' --meshcat-port 7000
```

Open `http://NUC_IP:7000`. Do not forward that port from a router or expose it
to the public internet.

## A first sensor experiment

Copy `scenarios/custom_sensors.toml`, change its scenario name, output values,
bias, or sensor mounting, then keep the original file as a known-good example.
Validate before running:

```bash
.venv/bin/blue-drake validate my_sensor_experiment.toml
.venv/bin/blue-drake inspect my_sensor_experiment.toml
.venv/bin/blue-drake run my_sensor_experiment.toml \
  --no-visualizer --realtime-rate 0 \
  --output-dir runs/my-sensor-001 --log-period 0.02
```

The output directory is never overwritten. Compare the manifest's software
versions and scenario summary before comparing CSV data from separate runs.
This workflow is intended to catch integration, units, mounting, bounds, and
timing mistakes before hardware or water testing; it does not qualify the
sensor or predict field accuracy.

## Common failures

- **`No module named pydrake`**: install `.[drake-current]` inside the active
  virtual environment and rerun `doctor`.
- **Meshcat port unavailable**: omit `--meshcat-port` to select a free local
  port, or stop the process already using the requested fixed port.
- **A body starts in contact**: run `validate`; scenarios reject oriented
  bounding boxes that begin inside the flat seafloor.
- **A run directory already exists**: choose a new experiment ID. Blue Drake
  intentionally does not append or overwrite evidence.
- **A long run consumes memory**: state and sensor logs are retained until the
  run finishes. Increase `--log-period`, shorten the run, or omit artifacts.
