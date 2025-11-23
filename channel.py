"""
Wireless Channel simulation for Drone Mesh Network
"""

import asyncio
import math
import random
from typing import Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from drone_node import DroneNode


class WirelessChannel:
    """In-memory 'air' that delivers frames to nodes in range with delay/jitter."""
    
    def __init__(self, comm_range: float, cfg: Dict[str, Any]):
        self.comm_range = comm_range
        self.cfg = cfg
        self.nodes: Dict[int, "DroneNode"] = {}

    def attach(self, node: "DroneNode"):
        """Attach a drone node to the wireless channel"""
        self.nodes[node.nid] = node

    async def broadcast(self, sender_id: int, msg: Any):
        """Broadcast a message to all nodes within communication range"""
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
        """Send a message to a specific node (unicast)"""
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
        """Deliver a message to a node after a specified delay"""
        await asyncio.sleep(delay)
        await node.inbox.put(msg)
