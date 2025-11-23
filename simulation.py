"""
Simulation coordinator for Drone Mesh Network
"""

import asyncio
from typing import Dict, List, Any

from channel import WirelessChannel
from drone_node import DroneNode


class Simulation:
    """Coordinates the entire drone mesh network simulation"""
    
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.channel = WirelessChannel(cfg["comm_range"], cfg)
        self.nodes: List[DroneNode] = []
        self.tasks: List[asyncio.Task] = []
        self._running = False

    def build(self):
        """Create and initialize all drone nodes"""
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
        """Run the simulation for the configured duration"""
        self._running = True
        all_ids = [n.nid for n in self.nodes]
        
        # Start all node tasks
        for n in self.nodes:
            self.tasks.append(asyncio.create_task(n.mobility_task()))
            self.tasks.append(asyncio.create_task(n.hello_task()))
            self.tasks.append(asyncio.create_task(n.dv_task()))
            self.tasks.append(asyncio.create_task(n.rx_loop()))
            self.tasks.append(asyncio.create_task(n.app_task(all_ids)))

        # Run for specified simulation time
        await asyncio.sleep(self.cfg["sim_time_s"])

        # Cancel all tasks
        for t in self.tasks:
            t.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self._running = False

    def report(self):
        """Print simulation statistics and results"""
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
