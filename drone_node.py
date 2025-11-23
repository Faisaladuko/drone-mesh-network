"""
Drone Node implementation for Mesh Network
"""

import asyncio
import math
import random
import time
from typing import Dict, Optional, Tuple, List, Set, Any

from channel import WirelessChannel
from messages import HelloMsg, DVMsg, DataMsg
from routing import Route, ensure_one_hop, apply_distance_vector


class DroneNode:
    """A mobile drone node with routing and data forwarding capabilities"""
    
    def __init__(self, nid: int, channel: WirelessChannel, cfg: Dict[str, Any],
                 world_size: Tuple[float, float], z_level: float):
        self.nid = nid
        self.cfg = cfg
        self.channel = channel
        self.world_size = world_size
        self.z = z_level

        # Initial random position
        self.pos: Tuple[float, float, float] = (
            random.uniform(0, world_size[0]),
            random.uniform(0, world_size[1]),
            self.z
        )
        
        # Mobility state
        self._target_wp: Optional[Tuple[float, float]] = None
        self._wp_pause_until: float = 0.0
        self._speed = random.uniform(*cfg["speed_mps"])

        # Communication
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
        """Select a new random waypoint for the drone"""
        x = random.uniform(0, self.world_size[0])
        y = random.uniform(0, self.world_size[1])
        self._target_wp = (x, y)
        self._speed = random.uniform(*self.cfg["speed_mps"])
        pause = random.uniform(*self.cfg["waypoint_pause_s"])
        self._wp_pause_until = time.time() + pause

    def _step_toward_waypoint(self, dt: float):
        """Move drone toward current waypoint"""
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
        """Asyncio task for continuous mobility"""
        self._pick_new_waypoint()
        tick = self.cfg["mobility_step_s"]
        while True:
            self._step_toward_waypoint(tick)
            await asyncio.sleep(tick)

    # -------- Neighbor Discovery & DV --------

    async def hello_task(self):
        """Periodic HELLO beacon broadcasts for neighbor discovery"""
        period = self.cfg["hello_period_s"]
        while True:
            self._hello_seq += 1
            await self.channel.broadcast(self.nid, HelloMsg(self.nid, self.pos, self._hello_seq))
            await asyncio.sleep(period)

    async def dv_task(self):
        """Periodic Distance Vector routing updates"""
        period = self.cfg["dv_period_s"]
        while True:
            self._dv_seq += 1
            vector = {dest: (route.cost, route.next_hop) for dest, route in self.rt.items()}
            await self.channel.broadcast(self.nid, DVMsg(self.nid, vector, self._dv_seq))
            await asyncio.sleep(period)

    # -------- Data Plane --------

    async def app_task(self, all_ids: List[int]):
        """Application layer: generate random data packets"""
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
        """Forward a data packet toward its destination"""
        if msg.dst == self.nid:
            # Packet reached destination
            self.delivered += 1
            latency = time.time() - msg.created_at
            self.latencies.append(latency)
            self.hops_used.append(msg.hop_count)
            return
        
        # Look up route and forward
        route = self.rt.get(msg.dst)
        if route is None:
            return  # No route available
        msg.hop_count += 1
        await self.channel.unicast(self.nid, route.next_hop, msg)

    # -------- RX Loop --------

    async def rx_loop(self):
        """Main receive loop: process incoming messages"""
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
        """Return node statistics summary"""
        return {
            "nid": self.nid,
            "generated": self.generated,
            "delivered": self.delivered,
            "delivery_ratio": (self.delivered / self.generated) if self.generated else 0.0,
            "avg_latency_s": (sum(self.latencies) / len(self.latencies)) if self.latencies else None,
            "avg_hops": (sum(self.hops_used) / len(self.hops_used)) if self.hops_used else None,
            "neighbors_now": sorted(self.neighbors),
            "routes_now": {d: (r.next_hop, round(r.cost, 1)) for d, r in sorted(self.rt.items())},
        }
