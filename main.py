#!/usr/bin/env python3
"""
Drone Mesh Network Simulator - Main Entry Point

A modular simulation of a mobile ad-hoc mesh network for drones with live visualization.

Features:
- Asyncio nodes with waypoint mobility
- Neighbor discovery via HELLO beacons
- Distance-vector routing (Bellman-Ford style)
- Multi-hop data forwarding
- In-memory wireless channel with range + delay/jitter
- Live Matplotlib animation: moving nodes + neighbor links + stats banner

Run:
    python drone.py
"""

from config import SIM_CONFIG
from simulation import Simulation
from visualization import run_live_viz


def main():
    """Main entry point for the drone mesh network simulation"""
    sim = Simulation(SIM_CONFIG)
    sim.build()
    print("Starting simulation with live visualization...")
    run_live_viz(sim)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
