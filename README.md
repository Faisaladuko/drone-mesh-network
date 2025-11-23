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

## Configuration

Key simulation parameters (in `SIM_CONFIG`):
- **Nodes**: 14 drones
- **World Size**: 900m × 620m
- **Communication Range**: 230 meters
- **Simulation Time**: 35 seconds
- **Speed**: 10-22 m/s

## Requirements

```bash
pip install matplotlib
```

Python 3.7+ required (uses asyncio and dataclasses)

## Usage

Run the simulation with live visualization:

```bash
python drone.py
```

The visualization displays:
- Blue dots: drone positions
- Green lines: active communication links between neighbors
- Stats banner: real-time metrics on packet delivery, latency, and hop counts

## How It Works

1. **Mobility**: Drones move autonomously between random waypoints
2. **Discovery**: Periodic HELLO broadcasts detect neighbors within range
3. **Routing**: Distance vector protocol builds routing tables for multi-hop paths
4. **Data Transfer**: Application layer generates random data packets between drone pairs
5. **Visualization**: Updates in real-time showing network topology and performance

## Simulation Output

The simulation tracks:
- Packets generated and delivered
- End-to-end latency
- Hop counts for multi-hop routes
- Network connectivity changes over time
