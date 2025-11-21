# COMM_INTERFACE and RL Online Tuning

## Overview
This project provides tools for real‑time plotting and online reinforcement‑learning (RL) tuning of a prosthetic knee interface.

## Prerequisites
- Python 3.10 (use a conda environment)
- create py3.10 conda env, build and install the ossur_wireless_protocol_library from ossur_hf:
    ```
    cd ossur_wireless_protocol_library
    pip install -e .
    ```

- download and install LSPI RL lib
    ```
    git clone https://github.com/liuxy10/rl-lspi.git
    cd rl-lspi
    pip install -r requirements.txt
    pip install -e .
    ```
- clone source code repo, then:

    ```
    cd udp
    pip install -r requirement.txt
    pip install -e .
    ```

## Quick start

1. Real‑time plotting (demo)
     - Runs a simple real‑time position plot for incoming test data:
     ```
     python src/plot_real_time.py
     ```

2. UART logging 
    
    - real-time streaming of the gait trajectories from the wired communication
    python src/connection/udp.py


3. Online RL tuning
     - Start two terminals:
     ```
     # Terminal A: RL node
     python src/node_RL.py

     # Terminal B: Device interface node
     python src/node_interface.py
     ```

## Project layout
(Top-level files and key modules)
```
README.md                # This file
requirements.txt         # Python dependencies

env/                     # Environment and config
    bionics.json           # Main bionics settings
    online_cfg.json        # Online configuration presets
    maps.py                # Environment map config
    env.py                 # Gym online tuning environment
    var_names.json         # Variable name references

src/                     # Main source code
    config.py
    plot_real_time.py      # Real-time plotting demo
    node_RL.py             # RL node
    node_interface.py      # Device communication interface node
    ossur_knee_tester.py   # UI for tester (unused)
    adaptive_lqr_master.py # Adaptive LQR (unused)
    utils.py               # Helper utilities

src/connection/          # Communication layers
    udp.py
    tcp.py                 # (inefficient; unused)
    device.py
    receivers.py
    senders.py

experiment/              # Scripts for experiments and data collection
    online_tuning_*.py
    data_collection_*.py

tests/                   # Tests and emulators
    connection/
        emulator.py
        test_tcp_connection.py
```

