# Drone Mesh Network Simulator

A Python-based simulation of a mobile ad-hoc mesh network for drones with live visualization.

## Features

- **Mobile Drone Nodes**: 14 autonomous drones moving through waypoints in 3D space
- **Neighbor Discovery**: HELLO beacon protocol for detecting nearby drones
- **Distance Vector Routing**: Bellman-Ford style routing for multi-hop communication
- **Wireless Channel Simulation**: Range-limited communication with realistic delay and jitter
- **Live Visualization**: Real-time Matplotlib animation showing:
  - Drone positions and movement
  - Active neighbor links
  - Network statistics (packets delivered, latency, hop counts)

## Project Structure

The project is organized into modular components:

```
├── main.py            # Entry point for running the simulation
├── config.py          # Simulation configuration parameters
├── messages.py        # Message types (HELLO, DV, DATA)
├── channel.py         # Wireless channel simulation
├── routing.py         # Distance vector routing logic
├── drone_node.py      # Drone node implementation
├── simulation.py      # Simulation coordinator
├── visualization.py   # Live Matplotlib visualization
└── README.md          # This file
```

## Configuration

Key simulation parameters (in `config.py`):
- **Nodes**: 14 drones
- **World Size**: 900m × 620m
- **Communication Range**: 230 meters
- **Simulation Time**: 35 seconds
- **Speed**: 10-22 m/s
- **HELLO Period**: 0.6 seconds
- **DV Update Period**: 1.2 seconds

Modify `config.py` to customize simulation parameters.

## Requirements

```bash
pip install matplotlib
```

Python 3.7+ required (uses asyncio and dataclasses)

## Usage

Run the simulation with live visualization:

```bash
python main.py
```

The visualization displays:
- Blue dots: drone positions
- Green lines: active communication links between neighbors
- Stats banner: real-time metrics on packet delivery, latency, and hop counts

## How It Works

1. **Mobility** (`drone_node.py`): Drones move autonomously between random waypoints
2. **Discovery** (`drone_node.py`): Periodic HELLO broadcasts detect neighbors within range
3. **Routing** (`routing.py`): Distance vector protocol builds routing tables for multi-hop paths
4. **Channel** (`channel.py`): Simulates wireless propagation with range limits and realistic delays
5. **Data Transfer** (`drone_node.py`): Application layer generates random data packets between drone pairs
6. **Visualization** (`visualization.py`): Updates in real-time showing network topology and performance

## Simulation Output

The simulation tracks:
- Packets generated and delivered
- End-to-end latency
- Hop counts for multi-hop routes
- Network connectivity changes over time
- Per-node statistics and routing tables

## Architecture

### Modules

- **config.py**: Centralized configuration with all simulation parameters
- **messages.py**: Dataclass definitions for network messages
- **channel.py**: Simulates wireless medium with distance-based range checking and propagation delays
- **routing.py**: Implements Bellman-Ford distance vector routing algorithm
- **drone_node.py**: Core node logic including mobility, neighbor discovery, routing, and data forwarding
- **simulation.py**: Orchestrates the simulation lifecycle and collects metrics
- **visualization.py**: Real-time Matplotlib animation with network statistics
- **main.py**: Entry point that initializes and runs the simulation
