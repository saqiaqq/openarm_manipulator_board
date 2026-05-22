"""request_id 幂等 LRU 缓存。

同一 request_id 重复到达时，直接返回缓存结果，不重复执行命令。
容量满后淘汰最久未访问的条目。
"""

from __future__ import annotations

import threading
from collections import OrderedDict
from typing import Any, Optional


class IdempotencyCache:
    """线程安全的 LRU 缓存，以 request_id 为键。"""

    def __init__(self, capacity: int = 256) -> None:
        self._store: OrderedDict[str, Any] = OrderedDict()
        self._cap = max(1, capacity)
        self._lock = threading.Lock()

    def contains(self, key: str) -> bool:
        with self._lock:
            return key in self._store

    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            if key not in self._store:
                return None
            self._store.move_to_end(key)
            return self._store[key]

    def put(self, key: str, value: Any) -> None:
        with self._lock:
            if key in self._store:
                self._store.move_to_end(key)
            self._store[key] = value
            if len(self._store) > self._cap:
                self._store.popitem(last=False)

    def __len__(self) -> int:
        with self._lock:
            return len(self._store)
