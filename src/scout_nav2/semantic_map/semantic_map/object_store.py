"""In-memory semantic object database with data association.

Pure logic, free of ROS imports, so the association/merge behaviour can be unit
tested standalone (see test/test_object_store.py). The node layer
(semantic_map_node.py) wraps this with subscriptions, a service and markers.

JSON persistence (M5, SPEC 2.3): `save`/`load` round-trip the store to disk so
the map survives a restart. Objects are treated as static landmarks (never
deleted), so loading simply re-inserts everything with fresh marker ids.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional


@dataclass
class StoredObject:
    label: str
    x: float
    y: float
    z: float
    confidence: float
    last_seen: float  # seconds (e.g. ROS time as float)
    count: int = 1


class ObjectStore:
    """Accumulates detections, merging nearby same-label observations.

    Association rule (SPEC 2.3): a new observation merges into an existing
    object when it shares the label and lies within `merge_distance`; the merged
    position is an exponential moving average. Otherwise it becomes a new object.
    """

    def __init__(self, merge_distance: float = 0.5, ema_alpha: float = 0.3):
        self.merge_distance = float(merge_distance)
        self.ema_alpha = float(ema_alpha)
        self._objects: Dict[int, StoredObject] = {}
        self._next_id = 0

    def update(
        self, label: str, position, confidence: float, stamp: float
    ) -> StoredObject:
        """Insert or merge one observation. `position` is an (x, y, z) tuple."""
        px, py, pz = (float(position[0]), float(position[1]), float(position[2]))
        match = self._nearest_same_label(label, px, py, pz)
        if match is None:
            obj = StoredObject(
                label=label,
                x=px,
                y=py,
                z=pz,
                confidence=float(confidence),
                last_seen=float(stamp),
            )
            self._objects[self._next_id] = obj
            self._next_id += 1
            return obj

        a = self.ema_alpha
        match.x = (1.0 - a) * match.x + a * px
        match.y = (1.0 - a) * match.y + a * py
        match.z = (1.0 - a) * match.z + a * pz
        match.confidence = float(confidence)
        match.last_seen = float(stamp)
        match.count += 1
        return match

    def _nearest_same_label(
        self, label: str, px: float, py: float, pz: float
    ) -> Optional[StoredObject]:
        best = None
        best_dist = self.merge_distance
        for obj in self._objects.values():
            if obj.label != label:
                continue
            dist = math.sqrt(
                (obj.x - px) ** 2 + (obj.y - py) ** 2 + (obj.z - pz) ** 2
            )
            if dist < best_dist:
                best = obj
                best_dist = dist
        return best

    def find(self, label: str, min_count: int = 1) -> List[StoredObject]:
        """All instances of a label seen at least `min_count` times.

        Returns every matching object (so callers can disambiguate multiple
        same-label instances), sorted by confidence descending. `min_count`
        filters out unconfirmed objects (a one-off false positive never reaches
        the observation count, so confirmation acts as a natural noise filter).
        """
        matches = [
            o
            for o in self._objects.values()
            if o.label == label and o.count >= min_count
        ]
        matches.sort(key=lambda o: o.confidence, reverse=True)
        return matches

    def all(self) -> List[StoredObject]:
        return list(self._objects.values())

    def items(self):
        """(id, StoredObject) pairs; ids are stable marker ids."""
        return list(self._objects.items())

    def save(self, path: str) -> int:
        """Write all objects to `path` as JSON (atomic). Returns the count.

        Writes to a temp file then renames so a crash mid-write can't leave a
        truncated map. Creates parent directories as needed.
        """
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        objs = [asdict(o) for o in self._objects.values()]
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"objects": objs}, f, indent=2)
        os.replace(tmp, path)
        return len(objs)

    def load(self, path: str) -> int:
        """Load objects from a JSON file written by `save`. Returns the count.

        Missing file -> 0 (first run). Loaded objects are appended with fresh
        ids, so loading into a non-empty store merges by the usual rules is NOT
        done here -- it is a straight restore meant for an empty store at start.
        """
        if not os.path.exists(path):
            return 0
        with open(path) as f:
            data = json.load(f)
        count = 0
        for d in data.get("objects", []):
            self._objects[self._next_id] = StoredObject(
                label=str(d["label"]),
                x=float(d["x"]),
                y=float(d["y"]),
                z=float(d["z"]),
                confidence=float(d["confidence"]),
                last_seen=float(d["last_seen"]),
                count=int(d.get("count", 1)),
            )
            self._next_id += 1
            count += 1
        return count
