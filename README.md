# AMR Sweeper FSM

This repository contains the **finite state machine (FSM) supervisor** for the AMR Sweeper platform.
It is responsible for orchestrating high-level robot behavior by managing **ROS 2 lifecycle state nodes**, launching and supervising processes, and enforcing readiness and fault-handling rules.

The FSM is designed for **robust autonomous operation**, **clear fault containment**, and **deployment-grade configurability**.

---

## Architecture Overview

The FSM consists of:

### 1. Supervisor Node
- Central decision-making engine
- Executes the FSM logic on a periodic **tick**
- Drives lifecycle transitions between states
- Monitors state health and lifecycle labels
- Publishes FSM status and diagnostics

### 2. State Nodes (Lifecycle Nodes)
Each FSM state is implemented as a ROS 2 **LifecycleNode**, for example:
- `idle`
- `navigation`
- `docking`
- `charging`
- `fault`

Each state node:
- Loads a **profile-specific configuration**
- Launches and supervises one or more processes
- Enforces readiness conditions
- Reacts to log-based triggers (e.g. ERROR → FAULT)

### 3. Profiles
Profiles define **what runs and how** in each state.

A profile typically specifies:
- Processes to start/stop
- Readiness conditions (topics, services, parameters)
- Restart behavior
- Fault and degradation triggers

Profiles are defined in YAML and selected at launch time.

---

## FSM Tick

The supervisor runs a periodic **tick loop** that:
1. Evaluates the current FSM state
2. Schedules at most one lifecycle action per tick
3. Polls the active state's lifecycle label

The tick period is configurable and defaults to **100 ms**.

> Slower ticks increase transition latency but do not break correctness.

---

## Package Layout

```
amr_sweeper_fsm/
├── src/
│   └── _supervisor/        # Supervisor node implementation
├── include/
│   └── _supervisor/        # Supervisor headers
├── launch/
│   └── amr_sweeper_fsm.launch.py
├── config/
│   └── state_parameters.yaml
├── profiles/
│   ├── default/
│   ├── simulation/
│   └── production/
└── README.md
```

---

## Configuration

### State Parameters File

The FSM uses a global **state parameters YAML** to define available states and their properties.

By default, this file is resolved relative to the package share directory:

```
<package_share>/config/state_parameters.yaml
```

This path is resolved at runtime using the ROS 2 ament index, making it portable across:
- overlay workspaces
- system installs
- containers
- custom install prefixes

---

### Profiles

Profiles live under:

```
<package_share>/profiles/<profile_name>/
```

Each profile may define:
- process lists
- readiness rules
- log triggers
- restart and fault behavior

---

## Launching the FSM

### Basic launch
```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py
```

### Select a profile
```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py profile:=simulation
```

### Configure tick period (example: 5 seconds)
```bash
ros2 launch amr_sweeper_fsm amr_sweeper_fsm.launch.py tick_period_ms:=5000
```

---

## Launch Arguments

| Argument            | Description                            | Default |
|---------------------|----------------------------------------|---------|
| `profile`           | FSM profile name                       | `default` |
| `tick_period_ms`    | Supervisor tick period in milliseconds| `100` |
| `state_params_file` | Path to FSM state parameters YAML      | `<package_share>/config/state_parameters.yaml` |

---

## Runtime Introspection

### Check active profile
```bash
ros2 param get /supervisor profile
```

### Check FSM state
```bash
ros2 topic echo /fsm/state
```

### Check lifecycle state of a state node
```bash
ros2 lifecycle get /navigation_state
```

---

## Fault Handling

The FSM supports:
- automatic fault transitions based on log triggers
- controlled shutdown and cleanup of processes
- isolation of faulty subsystems

---

## Design Principles

- Deterministic lifecycle management
- Explicit fault containment
- Non-blocking supervisor logic
- Deployment-safe configuration
- No hidden side effects

The FSM is designed to be slow, observable, and correct — not clever.

---

## Notes for Deployment

- Always launch with an explicit `profile` in production.
- Avoid hard-coding absolute paths in configs.
- Keep tick period conservative unless rapid transitions are required.
- Validate profile YAML strictly before deployment.

---

## License

[Add license here]

---

## Maintainers

O-Robotics
