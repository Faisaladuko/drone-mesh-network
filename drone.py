#!/usr/bin/env python3
"""
Drone Mesh (All-in-One + Live Visualization)

- asyncio nodes with waypoint mobility
- neighbor discovery via HELLO beacons
- distance-vector routing (Bellman-Ford style)
- multi-hop data forwarding
- in-memory wireless channel with range + delay/jitter
- live Matplotlib animation: moving nodes + neighbor links + stats banner

Run:
    python drone_mesh_viz.py
"""

import asyncio
import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, List, Set, Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection


# ----------------------------- Config -----------------------------

SIM_CONFIG = {
    "num_nodes": 14,
    "world_size": (900.0, 620.0),  # meters (x_max, y_max)
    "z_level": 50.0,               # constant altitude for now
    "comm_range": 230.0,           # meters radio range
    "hello_period_s": 0.6,         # neighbor beacons
    "dv_period_s": 1.2,            # routing periodic updates
    "mobility_step_s": 0.20,       # mobility tick
    "app_send_period_s": 1.6,      # application DATA generation period
    "sim_time_s": 35.0,            # total simulation time
    "speed_mps": (10.0, 22.0),     # waypoint speed range
    "waypoint_pause_s": (0.0, 0.4),
    "channel_jitter_s": (0.002, 0.020),
    "channel_base_delay_s": 0.001, # added to distance delay
    "prop_speed_mps": 3e8,
    "max_per_hop_delay_s": 0.015,  # clamp per-hop delay so it’s visible
    "data_payload_bytes": 32,
    "app_pairs_per_period": 2,     # how many random (src,dst) pairs per app tick
    "seed": 42,
    "log_dv_changes": False,
}

random.seed(SIM_CONFIG["seed"])


# --------------------------- Message Types ------------------------

@dataclass
class HelloMsg:
    src: int
    pos: Tuple[float, float, float]
    seq: int

@dataclass
class DVMsg:
    src: int
    # distance vector: dest_id -> (cost, next_hop_from_src)
    vector: Dict[int, Tuple[float, int]]
    seq: int

@dataclass
class DataMsg:
    src: int
    dst: int
    payload: bytes
    created_at: float
    hop_count: int = 0
    id: int = field(default_factory=lambda: random.randint(1, 10_000_000))


# ----------------------------- Channel ----------------------------

class WirelessChannel:
    """In-memory 'air' that delivers frames to nodes in range with delay/jitter."""
    def __init__(self, comm_range: float, cfg: Dict[str, Any]):
        self.comm_range = comm_range
        self.cfg = cfg
        self.nodes: Dict[int, "DroneNode"] = {}

    def attach(self, node: "DroneNode"):
        self.nodes[node.nid] = node

    async def broadcast(self, sender_id: int, msg: Any):
        sender = self.nodes[sender_id]
        sx, sy, sz = sender.pos
        tasks = []
        for nid, node in self.nodes.items():
            if nid == sender_id:
                continue
            dist = math.dist((sx, sy, sz), node.pos)
            if dist <= self.comm_range:
                jitter = random.uniform(*self.cfg["channel_jitter_s"])
                dist_delay = min(dist / self.cfg["prop_speed_mps"], self.cfg["max_per_hop_delay_s"])
                delay = self.cfg["channel_base_delay_s"] + jitter + dist_delay
                tasks.append(asyncio.create_task(self._deliver_with_delay(node, msg, delay)))
        if tasks:
            await asyncio.gather(*tasks)

    async def unicast(self, sender_id: int, next_hop_id: int, msg: Any):
        sender = self.nodes[sender_id]
        if next_hop_id not in self.nodes:
            return
        rx = self.nodes[next_hop_id]
        dist = math.dist(sender.pos, rx.pos)
        if dist > self.comm_range:
            return
        jitter = random.uniform(*self.cfg["channel_jitter_s"])
        dist_delay = min(dist / self.cfg["prop_speed_mps"], self.cfg["max_per_hop_delay_s"])
        delay = self.cfg["channel_base_delay_s"] + jitter + dist_delay
        await self._deliver_with_delay(rx, msg, delay)

    async def _deliver_with_delay(self, node: "DroneNode", msg: Any, delay: float):
        await asyncio.sleep(delay)
        await node.inbox.put(msg)


# ----------------------------- Routing ----------------------------

@dataclass
class Route:
    cost: float
    next_hop: int
    updated_at: float


def ensure_one_hop(rt: Dict[int, Route], neighbor_id: int, *, log: bool = False):
    now = time.time()
    old = rt.get(neighbor_id)
    if old is None or old.cost > 1.0:
        rt[neighbor_id] = Route(cost=1.0, next_hop=neighbor_id, updated_at=now)
        if log:
            print(f"[DV] New 1-hop route to {neighbor_id}")


def apply_distance_vector(
    rt: Dict[int, Route],
    self_id: int,
    src: int,
    their_vector: Dict[int, Tuple[float, int]],
    *,
    log: bool = False,
):
    """Bellman-Ford relaxation: cost_via_src = 1 + their_cost"""
    ensure_one_hop(rt, src, log=log)
    now = time.time()
    for dest, (their_cost, _nh) in their_vector.items():
        if dest == self_id:
            continue
        cost_via_src = 1.0 + their_cost
        existing = rt.get(dest)
        if existing is None or cost_via_src + 1e-9 < existing.cost or existing.next_hop == src:
            rt[dest] = Route(cost=cost_via_src, next_hop=src, updated_at=now)
            if log:
                print(f"[DV] RT update: dest={dest} via {src}, cost={cost_via_src:.1f}")


# ----------------------------- Drone ------------------------------

class DroneNode:
    def __init__(self, nid: int, channel: WirelessChannel, cfg: Dict[str, Any],
                 world_size: Tuple[float, float], z_level: float):
        self.nid = nid
        self.cfg = cfg
        self.channel = channel
        self.world_size = world_size
        self.z = z_level

        self.pos: Tuple[float, float, float] = (
            random.uniform(0, world_size[0]),
            random.uniform(0, world_size[1]),
            self.z
        )
        self._target_wp: Optional[Tuple[float, float]] = None
        self._wp_pause_until: float = 0.0
        self._speed = random.uniform(*cfg["speed_mps"])

        self.inbox: asyncio.Queue = asyncio.Queue()
        self.neighbors: Set[int] = set()

        # DV Routing table: dest -> Route(cost, next_hop, updated_at)
        self.rt: Dict[int, Route] = {self.nid: Route(0.0, self.nid, time.time())}

        # Sequence counters
        self._hello_seq = 0
        self._dv_seq = 0

        # Metrics
        self.delivered: int = 0
        self.generated: int = 0
        self.latencies: List[float] = []
        self.hops_used: List[int] = []

    # -------- Mobility --------

    def _pick_new_waypoint(self):
        x = random.uniform(0, self.world_size[0])
        y = random.uniform(0, self.world_size[1])
        self._target_wp = (x, y)
        self._speed = random.uniform(*self.cfg["speed_mps"])
        pause = random.uniform(*self.cfg["waypoint_pause_s"])
        self._wp_pause_until = time.time() + pause

    def _step_toward_waypoint(self, dt: float):
        if self._target_wp is None or time.time() < self._wp_pause_until:
            return
        tx, ty = self._target_wp
        x, y, z = self.pos
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self._pick_new_waypoint()
            return
        step = self._speed * dt
        if step >= dist:
            self.pos = (tx, ty, self.z)
            self._pick_new_waypoint()
        else:
            self.pos = (x + step * dx / dist, y + step * dy / dist, self.z)

    async def mobility_task(self):
        self._pick_new_waypoint()
        tick = self.cfg["mobility_step_s"]
        while True:
            self._step_toward_waypoint(tick)
            await asyncio.sleep(tick)

    # -------- Neighbor Discovery & DV --------

    async def hello_task(self):
        period = self.cfg["hello_period_s"]
        while True:
            self._hello_seq += 1
            await self.channel.broadcast(self.nid, HelloMsg(self.nid, self.pos, self._hello_seq))
            await asyncio.sleep(period)

    async def dv_task(self):
        period = self.cfg["dv_period_s"]
        while True:
            self._dv_seq += 1
            vector = {dest: (route.cost, route.next_hop) for dest, route in self.rt.items()}
            await self.channel.broadcast(self.nid, DVMsg(self.nid, vector, self._dv_seq))
            await asyncio.sleep(period)

    # -------- Data Plane --------

    async def app_task(self, all_ids: List[int]):
        period = self.cfg["app_send_period_s"]
        while True:
            for _ in range(self.cfg["app_pairs_per_period"]):
                dst = random.choice(all_ids)
                if dst == self.nid:
                    continue
                if hasattr(random, "randbytes"):
                    payload = random.randbytes(self.cfg["data_payload_bytes"])
                else:
                    payload = bytes([random.randint(0, 255) for _ in range(self.cfg["data_payload_bytes"])])
                msg = DataMsg(src=self.nid, dst=dst, payload=payload, created_at=time.time())
                self.generated += 1
                await self._forward(msg)
            await asyncio.sleep(period)

    async def _forward(self, msg: DataMsg):
        if msg.dst == self.nid:
            self.delivered += 1
            latency = time.time() - msg.created_at
            self.latencies.append(latency)
            self.hops_used.append(msg.hop_count)
            return
        route = self.rt.get(msg.dst)
        if route is None:
            return
        msg.hop_count += 1
        await self.channel.unicast(self.nid, route.next_hop, msg)

    # -------- RX Loop --------

    async def rx_loop(self):
        while True:
            m = await self.inbox.get()
            if isinstance(m, HelloMsg):
                if m.src not in self.neighbors:
                    self.neighbors.add(m.src)
                ensure_one_hop(self.rt, m.src, log=self.cfg["log_dv_changes"])

            elif isinstance(m, DVMsg):
                apply_distance_vector(self.rt, self.nid, m.src, m.vector, log=self.cfg["log_dv_changes"])

            elif isinstance(m, DataMsg):
                await self._forward(m)

    # -------- Summary --------

    def summary(self) -> Dict[str, Any]:
        return {
            "nid": self.nid,
            "generated": self.generated,
            "delivered": self.delivered,
            "delivery_ratio": (self.delivered / self.generated) if self.generated else 0.0,
            "avg_latency_s": (sum(self.latencies) / len(self.latencies)) if self.latencies else None,
            "avg_hops": (sum(self.hops_used) / len(self.hops_used)) if self.hops_used else None,
            "neighbors_now": sorted(self.neighbors),
            "routes_now": {d: (r.next_hop, round(r.cost,1)) for d, r in sorted(self.rt.items())},
        }


# --------------------------- Simulation ---------------------------

class Simulation:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.channel = WirelessChannel(cfg["comm_range"], cfg)
        self.nodes: List[DroneNode] = []
        self.tasks: List[asyncio.Task] = []
        self._running = False

    def build(self):
        W, H = self.cfg["world_size"]
        for nid in range(self.cfg["num_nodes"]):
            node = DroneNode(
                nid=nid,
                channel=self.channel,
                cfg=self.cfg,
                world_size=(W, H),
                z_level=self.cfg["z_level"]
            )
            self.channel.attach(node)
            self.nodes.append(node)

    async def run(self):
        self._running = True
        all_ids = [n.nid for n in self.nodes]
        for n in self.nodes:
            self.tasks.append(asyncio.create_task(n.mobility_task()))
            self.tasks.append(asyncio.create_task(n.hello_task()))
            self.tasks.append(asyncio.create_task(n.dv_task()))
            self.tasks.append(asyncio.create_task(n.rx_loop()))
            self.tasks.append(asyncio.create_task(n.app_task(all_ids)))

        await asyncio.sleep(self.cfg["sim_time_s"])

        for t in self.tasks:
            t.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self._running = False

    def report(self):
        tot_gen = sum(n.generated for n in self.nodes)
        tot_del = sum(n.delivered for n in self.nodes)
        all_lat = [lat for n in self.nodes for lat in n.latencies]
        all_hops = [h for n in self.nodes for h in n.hops_used]
        print("\n=== Simulation Summary ===")
        print(f"Nodes: {len(self.nodes)}  Range: {self.cfg['comm_range']} m  Duration: {self.cfg['sim_time_s']} s")
        print(f"Total generated: {tot_gen}  Total delivered: {tot_del}")
        dr = (tot_del / tot_gen) if tot_gen else 0.0
        print(f"Delivery ratio: {dr:.3f}")
        if all_lat:
            print(f"Avg latency: {sum(all_lat)/len(all_lat):.4f} s")
        if all_hops:
            print(f"Avg hops: {sum(all_hops)/len(all_hops):.3f}")

        print("\nPer-node quick view:")
        for n in self.nodes:
            s = n.summary()
            print(f"- Node {s['nid']}: delivered {s['delivered']}/{s['generated']} "
                  f"(dr={s['delivery_ratio']:.2f}), "
                  f"avg_lat={None if s['avg_latency_s'] is None else round(s['avg_latency_s'],4)}s, "
                  f"avg_hops={None if s['avg_hops'] is None else round(s['avg_hops'],2)}")
            compact = list(s["routes_now"].items())[:8]
            rt_str = ", ".join([f"{d}->({nh},c{c})" for d,(nh,c) in compact])
            print(f"  RT: {rt_str}{' ...' if len(s['routes_now'])>8 else ''}")


# ----------------------- Live Visualization -----------------------

def _compute_edges(nodes: List[DroneNode], comm_range: float):
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

        # Node scatter
        xs = [n.pos[0] for n in self.sim.nodes]
        ys = [n.pos[1] for n in self.sim.nodes]
        self.scatter = self.ax.scatter(xs, ys, s=46)

        # Labels
        self.labels = [self.ax.text(n.pos[0], n.pos[1] + 7, str(n.nid),
                                    ha="center", va="bottom", fontsize=8)
                       for n in self.sim.nodes]

        # Neighbor lines
        segs = _compute_edges(self.sim.nodes, self.sim.cfg["comm_range"])
        self.lines = LineCollection(segs, linewidths=0.9, alpha=0.6)
        self.ax.add_collection(self.lines)

        # Stats area
        self.stats_txt = self.ax.text(0.01, 0.99, "", transform=self.ax.transAxes, va="top")

        # Range ring legend (optional)
        # from matplotlib.patches import Circle
        # for n in self.sim.nodes[:1]:
        #     self.ax.add_patch(Circle((n.pos[0], n.pos[1]), self.sim.cfg["comm_range"],
        #                              fill=False, linewidth=0.4, alpha=0.25))

    def update(self, _frame):
        xs = [n.pos[0] for n in self.sim.nodes]
        ys = [n.pos[1] for n in self.sim.nodes]
        self.scatter.set_offsets(list(zip(xs, ys)))
        for lbl, x, y in zip(self.labels, xs, ys):
            lbl.set_position((x, y + 7))

        segs = _compute_edges(self.sim.nodes, self.sim.cfg["comm_range"])
        self.lines.set_segments(segs)

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
    """Run asyncio simulation in a background thread and show a live Matplotlib view."""
    def _run():
        asyncio.run(sim.run())

    t = threading.Thread(target=_run, daemon=True)
    t.start()

    artist = LiveArtist(sim)
    anim = FuncAnimation(artist.fig, artist.update, interval=100, blit=False)  # ~10 FPS

    # Show blocking window; after close, print final report
    plt.show()
    sim.report()


# ------------------------------ Main ------------------------------

def main():
    sim = Simulation(SIM_CONFIG)
    sim.build()
    print("Starting simulation with live visualization...")
    run_live_viz(sim)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass