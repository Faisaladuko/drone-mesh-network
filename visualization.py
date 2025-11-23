"""
Live visualization for Drone Mesh Network
"""

import asyncio
import math
import threading
from typing import List

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

from drone_node import DroneNode
from simulation import Simulation


def _compute_edges(nodes: List[DroneNode], comm_range: float):
    """Compute communication links between nodes within range"""
    segs = []
    n = len(nodes)
    for i in range(n):
        xi, yi, _ = nodes[i].pos
        for j in range(i + 1, n):
            xj, yj, _ = nodes[j].pos
            if (xi - xj) ** 2 + (yi - yj) ** 2 <= comm_range ** 2:
                segs.append(((xi, yi), (xj, yj)))
    return segs


class LiveArtist:
    """Matplotlib visualization of the drone mesh network"""
    
    def __init__(self, sim: Simulation):
        self.sim = sim
        self.fig, self.ax = plt.subplots(figsize=(8.6, 6.0))
        W, H = self.sim.cfg["world_size"]
        self.ax.set_xlim(0, W)
        self.ax.set_ylim(0, H)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Drone Mesh — Live View")

        # Node scatter plot
        xs = [n.pos[0] for n in self.sim.nodes]
        ys = [n.pos[1] for n in self.sim.nodes]
        self.scatter = self.ax.scatter(xs, ys, s=46)

        # Node ID labels
        self.labels = [self.ax.text(n.pos[0], n.pos[1] + 7, str(n.nid),
                                    ha="center", va="bottom", fontsize=8)
                       for n in self.sim.nodes]

        # Neighbor links (line collection)
        segs = _compute_edges(self.sim.nodes, self.sim.cfg["comm_range"])
        self.lines = LineCollection(segs, linewidths=0.9, alpha=0.6)
        self.ax.add_collection(self.lines)

        # Stats banner
        self.stats_txt = self.ax.text(0.01, 0.99, "", transform=self.ax.transAxes, va="top")

    def update(self, _frame):
        """Update animation frame"""
        # Update node positions
        xs = [n.pos[0] for n in self.sim.nodes]
        ys = [n.pos[1] for n in self.sim.nodes]
        self.scatter.set_offsets(list(zip(xs, ys)))
        
        # Update labels
        for lbl, x, y in zip(self.labels, xs, ys):
            lbl.set_position((x, y + 7))

        # Update neighbor links
        segs = _compute_edges(self.sim.nodes, self.sim.cfg["comm_range"])
        self.lines.set_segments(segs)

        # Update statistics
        tot_gen = sum(n.generated for n in self.sim.nodes)
        tot_del = sum(n.delivered for n in self.sim.nodes)
        dr = (tot_del / tot_gen) if tot_gen else 0.0
        all_hops = [h for n in self.sim.nodes for h in n.hops_used]
        avg_hops = (sum(all_hops) / len(all_hops)) if all_hops else 0.0
        all_lat = [l for n in self.sim.nodes for l in n.latencies]
        avg_lat = (sum(all_lat) / len(all_lat)) if all_lat else 0.0

        self.stats_txt.set_text(
            f"Generated: {tot_gen}  Delivered: {tot_del}  DR: {dr:.2f}\n"
            f"Avg hops: {avg_hops:.2f}  Avg latency: {avg_lat:.3f}s"
        )
        return self.scatter, self.lines, *self.labels, self.stats_txt


def run_live_viz(sim: Simulation):
    """Run asyncio simulation in a background thread and show a live Matplotlib view"""
    def _run():
        asyncio.run(sim.run())

    t = threading.Thread(target=_run, daemon=True)
    t.start()

    artist = LiveArtist(sim)
    anim = FuncAnimation(artist.fig, artist.update, interval=100, blit=False)  # ~10 FPS

    # Show blocking window; after close, print final report
    plt.show()
    sim.report()
