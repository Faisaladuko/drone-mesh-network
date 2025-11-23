"""
Message types for Drone Mesh Network
"""

import random
from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class HelloMsg:
    """Neighbor discovery beacon message"""
    src: int
    pos: Tuple[float, float, float]
    seq: int


@dataclass
class DVMsg:
    """Distance Vector routing update message"""
    src: int
    # distance vector: dest_id -> (cost, next_hop_from_src)
    vector: Dict[int, Tuple[float, int]]
    seq: int


@dataclass
class DataMsg:
    """Application data packet for multi-hop forwarding"""
    src: int
    dst: int
    payload: bytes
    created_at: float
    hop_count: int = 0
    id: int = field(default_factory=lambda: random.randint(1, 10_000_000))
