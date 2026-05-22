"""协议报文解析与字段校验（MANIPULATOR_PROTOCOL 1.1）。

上位机下发两类消息：
  manipulator_start   -> 话题 ${NS}/start
  manipulator_command -> 话题 ${NS}/command
"""

from __future__ import annotations

import json
import time
from typing import Optional, Tuple

# 支持的命令字
SUPPORTED_COMMANDS = frozenset({
    "pick_place", "pick", "place",
    "home", "stop", "gripper", "get_status",
})

# 各消息类型的必填字段
_REQUIRED_START = {"request_id"}
_REQUIRED_COMMAND = {"request_id", "command"}


def parse_message(raw: str) -> Tuple[Optional[dict], Optional[str]]:
    """解析 JSON 字符串。返回 (dict, None) 成功，(None, error_str) 失败。"""
    if not raw or not raw.strip():
        return None, "empty message"
    try:
        msg = json.loads(raw)
    except Exception as exc:
        return None, f"invalid JSON: {exc}"
    if not isinstance(msg, dict):
        return None, "message must be a JSON object"
    return msg, None


def validate_start(msg: dict) -> Tuple[bool, str]:
    """校验 manipulator_start 消息。返回 (ok, error_str)，ok 时 error_str 为空。"""
    missing = _REQUIRED_START - msg.keys()
    if missing:
        return False, f"missing required fields: {sorted(missing)}"
    if not msg.get("request_id"):
        return False, "request_id must be non-empty"
    return True, ""


def validate_command(msg: dict) -> Tuple[bool, str]:
    """校验 manipulator_command 消息。返回 (ok, error_str)，ok 时 error_str 为空。"""
    missing = _REQUIRED_COMMAND - msg.keys()
    if missing:
        return False, f"missing required fields: {sorted(missing)}"
    if not msg.get("request_id"):
        return False, "request_id must be non-empty"
    cmd = msg.get("command", "")
    if not cmd:
        return False, "command must be non-empty"
    return True, ""


def now_ms() -> int:
    """返回当前毫秒级 Unix 时间戳（与上位机 timestamp 字段对齐）。"""
    return int(time.time() * 1000)
