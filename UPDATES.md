# Project Updates

This file documents the development progress and changes made to the `CatSwarm/general_infrastructure` project by the AI agent.

## [2026-01-20] ZMQ Command Tool
- **`multidrone/simpleZMQtakeoffland.py`**:
    - Refactored to use `argparse` for robust argument parsing.
    - Added `--takeoff` and `--land` mutually exclusive flags.
    - Added `--altitude` argument (default 10.0m).
    - Implemented `quadLandCmd` topic for landing commands.

## [2026-01-20] Command Tool Unification
- **`multidrone/takeoffland.py` & `multidrone/simpleZMQtakeoffland.py`**:
    - Unified the command-line interface for both scripts.
    - Both now support `--takeoff`, `--land`, and `--altitude=...` arguments.
    - `simpleZMQtakeoffland.py` uses `--zmq=...` for port configuration.
    - `takeoffland.py` uses `--udp=...` for port configuration.
    - Implemented `argparse` in `takeoffland.py` for standard argument handling and help display.

## [2026-01-20] Documentation
- **Added `README.md`**: Created project documentation listing files and architecture.
- **Added `UPDATES.md`**: Started tracking project history.

## [2026-01-19] Multi-Drone & Hardware Adapter
- **`multidrone/run_multidrone_bridges.sh`**: Created a new script to handle multiple drones.
    - Automatically parses `positions.txt` to count drones.
    - Orchestrates a tmux session with dedicated windows for each drone.
    - Launches paired `mavlink_to_ZMQ` and `zmq_commands_mavlink` binaries with calculated ports (UDP/ZMQ).
- **`hardware_adapter.sh`**: Refined the single-drone adapter script.
- **`runSimNoeticMulti.sh`**: Updated to support mounting custom `positions.txt` files into the Docker container.

## [2026-01-XX] UI & Refinement (Related Context)
- **GUI Refinement**: Improved `CatSwarm` GUI (in parallel workspace) to handle `.csm` project files and updated control labels.
- **Drone Configuration**: Updated UI to support table-based configuration of initial positions.

## [2025-12-22] Hardware Adapter Async Support
- **`hardware_adapter.py`**: Refactored to use asynchronous threads for sending and receiving MAVLink messages. This separated the listener and commander loops to prevent blocking.

## [2025-11-19] System Identification & Timing
- **`sysid.py`**: Converted the system identification logic from a Jupyter Notebook (`sysid.ipynb`) to a standalone Python script for better version control and execution.
- **`system_manager.py`**: Implemented precise loop timing. Replaced simple `time.sleep()` with a drift-correction mechanism to ensure exactly 100Hz (0.01s) loop execution, accounting for processing time.
