# Fixed-step simulation

The continuous game mode and the asset-free `nav-headless` command call
`simulation::step_continuous`. The ECS adapter snapshots `world::Unit` data in
stable ID order, advances the core once, then writes back once. It contains no
steering or contact algorithm. `Runtime` owns the fixed timestep and input seed;
the game configures Bevy's fixed clock from that same duration.

`Crowd=0` retains the discrete comparison mode; `Crowd=1` selects the continuous
mode. Both update functions guard their mode, including when called outside the
game schedule. A command has one monotonic ID shared by all selected units;
ordering units by stable ID makes ECS insertion order irrelevant.

An asset-free smoke run, from the project root:

```text
cargo run --locked -- nav-headless --ticks 240 --seed 42 --hz 60 --sample-every 60
```

This creates a small synthetic grid, uses the production flow field and
continuous simulation, and never initializes a renderer or reads game assets.
The seed controls deterministic input perturbations. Repeatability applies to
the same build and inputs; timing observations are excluded from simulation
decisions. This is a behavior baseline, not a crossing or throughput acceptance
benchmark.

`Runtime::sample_every` enables in-memory sampled events. Each event carries the
tick, seed, stable unit and command IDs, command status, positions, velocities,
and timings for snapshot, pressure, velocity, contact, and writeback. Sampling
does not open files or read environment switches per tick. Callers decide how to
present or persist observations.

Arrival is **not ready**: `TickSample::arrived` is `None`. The existing proximity
heuristic, inactivity and turret idling do not mean that a command completed.
The existing steering, pressure rebuild cadence, stall escape and overlap
projection are retained, including their known collision and parking limitations.

Tests compare the ECS adapter and direct core on every tick at 30, 60 and 120 Hz,
check seed reproducibility and observational sampling, and prevent simultaneous
position writes by both modes. The initial extraction also matched five recorded
trajectory checkpoints in [the original refactor](https://github.com/KenosisM/BevyAlert/pull/2):
`6ed72c8db04e1557`, `4163f401b6614c01`, `6ee369b78fccbd06`,
`28632edbb38e9cc2`, `290d1f8f2fb3bb53` at ticks 1, 4, 31, 120 and 240.

These are historical fingerprints, not constraints on intentional navigation
fixes. Current navigation correctness is checked against an independently
constructed small-graph shortest-path reference and production movement tests;
see [navigation rules](NAVIGATION.md). The ECS/core equivalence and sampling
invariance checks continue to run on the current implementation.
