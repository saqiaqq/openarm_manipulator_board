"""manipulator_board_node -- 机械臂板卡协议适配节点。

架构
----
上位机
  --publish-->  ${NS}/start     (std_msgs/String JSON)
  --publish-->  ${NS}/command   (std_msgs/String JSON)
  --subscribe-- ${NS}/status    (std_msgs/String JSON)

本节点
  订阅 start / command
  调用 /openarm/pick_place (action)
       /openarm/stop        (service)
       /openarm/goto_home   (service)
       /openarm/gripper     (service)
  发布 status（周期 + 每次命令前后各一帧）

设计要点
--------
- request_id 幂等：重复到达的 request_id 不重复执行
- stop 最高优先级：任意时刻可打断当前 action
- 命令派发到后台线程（topic callback 快速返回）
- 同时只允许一个长动作（忙时拒绝新动作命令）
- 协议错误码通过 protocol_errors.skill_to_protocol 与 skills 内部码解耦
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion

from openarm_skills.action import PickPlace
from openarm_skills.srv import (
    Stop as StopSrv,
    GotoHome as GotoHomeSrv,
    Gripper as GripperSrv,
)

from . import protocol_errors as perr
from .idempotency_cache import IdempotencyCache
from .protocol import (
    SUPPORTED_COMMANDS,
    now_ms,
    parse_message,
    validate_command,
    validate_start,
)


# ---------------------------------------------------------------------------
# 位姿转换工具
# ---------------------------------------------------------------------------
def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _pose_dict_to_msg(d: Optional[dict]) -> Pose:
    p = Pose()
    if not d:
        return p
    x, y, z = d.get("xyz", [0.0, 0.0, 0.0])
    r, pi, ya = d.get("rpy", [0.0, 0.0, 0.0])
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation = _rpy_to_quat(float(r), float(pi), float(ya))
    return p


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
class ManipulatorBoardNode(Node):
    """机械臂板卡协议适配节点（MANIPULATOR_PROTOCOL 1.1）。"""

    def __init__(self) -> None:
        super().__init__("manipulator_board_node")

        # ---- 参数声明 --------------------------------------------------------
        self.declare_parameter(
            "topic_ns",
            os.environ.get("MANIPULATOR_TOPIC_NS", "/robot_arm"),
        )
        self.declare_parameter("board_id", "arm-controller-01")
        self.declare_parameter("status_publish_hz", 5.0)
        self.declare_parameter("default_arm", "right")
        self.declare_parameter("default_pose_source", "upper_computer")
        self.declare_parameter("command_timeout_s", 60.0)
        self.declare_parameter("idempotency_cache_size", 256)
        # enable=false 时是否仍允许普通 command（默认拒绝）
        self.declare_parameter("allow_motion_when_disabled", False)
        # 默认抓取/放置点（use_default_poses=true 且 params 中没有位姿时使用）
        self.declare_parameter("use_default_poses", False)
        self.declare_parameter("default_grasp_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("default_grasp_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("default_place_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("default_place_rpy", [0.0, 0.0, 0.0])

        ns: str = str(self.get_parameter("topic_ns").value)
        self._board_id: str = str(self.get_parameter("board_id").value)
        self._default_arm: str = str(self.get_parameter("default_arm").value)
        self._default_pose_source: str = str(
            self.get_parameter("default_pose_source").value
        )
        self._command_timeout_s: float = float(
            self.get_parameter("command_timeout_s").value
        )
        self._allow_motion_when_disabled: bool = bool(
            self.get_parameter("allow_motion_when_disabled").value
        )

        cache_size = int(self.get_parameter("idempotency_cache_size").value)
        self._cache = IdempotencyCache(capacity=cache_size)

        # ---- 内部状态（用 _state_lock 保护）---------------------------------
        self._state_lock = threading.Lock()
        self._control_enabled: bool = False
        self._state: str = "stopped"    # idle|running|paused|stopped|error
        self._running: bool = False
        self._ack: str = "done"         # accepted|running|done|rejected|timeout
        self._error_code: int = 0
        self._error_message: str = ""
        self._last_request_id: str = ""
        self._current_mode: str = "manual"

        # 当前 pick_place goal handle（stop 时用于 cancel）
        self._goal_handle: Optional[object] = None
        self._goal_handle_lock = threading.Lock()

        # 忙标志：同时只允许一个长动作
        self._busy = threading.Event()

        # ---- ROS2 接口 -------------------------------------------------------
        self._status_pub = self.create_publisher(String, f"{ns}/status", 10)
        self._start_sub = self.create_subscription(
            String, f"{ns}/start", self._on_start, 10
        )
        self._command_sub = self.create_subscription(
            String, f"{ns}/command", self._on_command, 10
        )

        self._pick_place_client = ActionClient(self, PickPlace, "/openarm/pick_place")
        self._stop_client = self.create_client(StopSrv, "/openarm/stop")
        self._home_client = self.create_client(GotoHomeSrv, "/openarm/goto_home")
        self._gripper_client = self.create_client(GripperSrv, "/openarm/gripper")

        # 周期状态发布定时器
        hz = float(self.get_parameter("status_publish_hz").value)
        period = 1.0 / max(hz, 0.1)
        self.create_timer(period, self._publish_status)

        self.get_logger().info(
            f"manipulator_board_node ready  "
            f"start={ns}/start  command={ns}/command  status={ns}/status"
        )

    # ======================================================================
    # 话题回调（快速返回，阻塞逻辑移到后台线程）
    # ======================================================================
    def _on_start(self, msg: String) -> None:
        data, err = parse_message(msg.data)
        if err:
            self.get_logger().warning(f"[start] parse error: {err}")
            return

        ok, verr = validate_start(data)
        rid: str = data.get("request_id", "")

        if not ok:
            self.get_logger().warning(f"[start] validate error: {verr}")
            self._set_status(rid, "stopped", False, "rejected", perr.PARAM_ERROR, verr)
            self._publish_status()
            return

        # 幂等检查
        if rid and self._cache.contains(rid):
            self.get_logger().info(f"[start] dedup rid={rid}")
            self._publish_status()
            return

        enable: bool = bool(data.get("enable", True))
        mode: str = str(data.get("mode", "manual"))
        self.get_logger().info(f"[start] enable={enable} mode={mode} rid={rid}")

        if enable:
            with self._state_lock:
                self._control_enabled = True
                self._current_mode = mode
                self._state = "idle"
                self._ack = "done"
                self._error_code = 0
                self._error_message = ""
                self._last_request_id = rid
            if rid:
                self._cache.put(rid, "done")
            self._publish_status()
        else:
            # enable=false：后台线程发 stop 再禁控
            threading.Thread(
                target=self._exec_disable, args=(rid,), daemon=True
            ).start()

    def _on_command(self, msg: String) -> None:
        data, err = parse_message(msg.data)
        if err:
            self.get_logger().warning(f"[command] parse error: {err}")
            return

        ok, verr = validate_command(data)
        rid: str = data.get("request_id", "")
        cmd: str = data.get("command", "")

        if not ok:
            self.get_logger().warning(f"[command] validate error: {verr}")
            self._set_status(rid, "error", False, "rejected", perr.PARAM_ERROR, verr)
            self._publish_status()
            if rid:
                self._cache.put(rid, "rejected")
            return

        # stop：最高优先级，不受幂等/忙限制
        if cmd == "stop":
            threading.Thread(
                target=self._exec_stop, args=(rid,), daemon=True
            ).start()
            return

        # get_status：立即回包，不执行动作
        if cmd == "get_status":
            with self._state_lock:
                self._last_request_id = rid
            self._publish_status()
            if rid:
                self._cache.put(rid, "done")
            return

        # 幂等检查
        if rid and self._cache.contains(rid):
            self.get_logger().info(f"[command] dedup rid={rid}")
            self._publish_status()
            return

        # 安全互锁
        if not self._control_enabled and not self._allow_motion_when_disabled:
            self.get_logger().warning(f"[command] rejected: control not enabled, rid={rid}")
            self._set_status(rid, "error", False, "rejected",
                             perr.SAFETY_INTERLOCK, "control not enabled; send start(enable=true) first")
            self._publish_status()
            if rid:
                self._cache.put(rid, "rejected")
            return

        # 忙检查
        if self._busy.is_set():
            self.get_logger().warning(f"[command] rejected: arm busy, rid={rid}")
            self._set_status(rid, "running", True, "rejected",
                             perr.ARM_BUSY, "arm is busy executing another command")
            self._publish_status()
            if rid:
                self._cache.put(rid, "rejected")
            return

        # 受理 -> 派发到后台线程
        self._set_status(rid, "running", True, "accepted", 0, "")
        self._publish_status()
        threading.Thread(target=self._exec_command, args=(data,), daemon=True).start()

    # ======================================================================
    # 后台执行线程
    # ======================================================================
    def _exec_disable(self, rid: str) -> None:
        """处理 enable=false：先 stop 再禁控。"""
        self._call_stop_service(rid)
        with self._state_lock:
            self._control_enabled = False
            self._state = "stopped"
            self._running = False
            self._ack = "done"
            self._error_code = 0
            self._error_message = ""
            self._last_request_id = rid
        if rid:
            self._cache.put(rid, "done")
        self._publish_status()
        self.get_logger().info(f"[start] disabled rid={rid}")

    def _exec_command(self, data: dict) -> None:
        """后台线程：执行单条动作命令（同时只允许一个）。"""
        self._busy.set()
        cmd: str = data.get("command", "")
        rid: str = data.get("request_id", "")
        params: dict = data.get("params", {}) or {}
        t0 = time.time()
        ec: int
        detail: str
        try:
            if cmd in ("pick_place", "pick", "place"):
                ec, detail = self._exec_pick_place(data, params)
            elif cmd == "home":
                ec, detail = self._exec_home(data, params)
            elif cmd == "gripper":
                ec, detail = self._exec_gripper(data, params)
            else:
                ec, detail = perr.CMD_NOT_SUPPORTED, f"unsupported command '{cmd}'"

            elapsed_ms = int((time.time() - t0) * 1000)
            success = (ec == 0)
            new_state = "idle" if success else "error"
            new_ack = "done" if success else "rejected"
            self.get_logger().info(
                f"[exec] cmd={cmd} rid={rid} ack={new_ack} "
                f"ec={ec}({perr.name_of(ec)}) elapsed={elapsed_ms}ms msg={detail}"
            )
            self._set_status(rid, new_state, False, new_ack, ec, detail)
            if rid:
                self._cache.put(rid, new_ack)
        finally:
            self._busy.clear()
            self._publish_status()

    def _exec_stop(self, rid: str) -> None:
        """执行 stop（最高优先级，随时可调）。"""
        t0 = time.time()
        # 取消当前 action goal
        with self._goal_handle_lock:
            gh = self._goal_handle
        if gh is not None:
            cancel_future = gh.cancel_goal_async()
            ev = threading.Event()
            cancel_future.add_done_callback(lambda _: ev.set())
            ev.wait(timeout=2.0)

        ec, detail = self._call_stop_service(rid)
        elapsed_ms = int((time.time() - t0) * 1000)
        self.get_logger().info(
            f"[exec] cmd=stop rid={rid} ec={ec} elapsed={elapsed_ms}ms"
        )
        with self._state_lock:
            self._running = False
            self._state = "stopped"
            self._ack = "done"
            self._error_code = ec
            self._error_message = detail
            self._last_request_id = rid
        if rid:
            self._cache.put(rid, "done")
        self._publish_status()

    # ======================================================================
    # 命令实现
    # ======================================================================
    def _exec_pick_place(self, data: dict, params: dict) -> Tuple[int, str]:
        cmd: str = data.get("command", "pick_place")

        # pick / place 单独操作：暂不支持（openarm_skills 只有完整流程）
        if cmd in ("pick", "place"):
            return (
                perr.CMD_NOT_SUPPORTED,
                f"'{cmd}' standalone is not supported; use 'pick_place' for full cycle",
            )

        # 位姿解析
        grasp_d: Optional[dict] = params.get("grasp_pose")
        place_d: Optional[dict] = params.get("place_pose")
        pose_source: str = (
            params.get("pose_source")
            or data.get("pose_source")
            or self._default_pose_source
        )

        if pose_source != "camera" and (not grasp_d or not place_d):
            if bool(self.get_parameter("use_default_poses").value):
                grasp_xyz = list(self.get_parameter("default_grasp_xyz").value)
                grasp_rpy = list(self.get_parameter("default_grasp_rpy").value)
                place_xyz = list(self.get_parameter("default_place_xyz").value)
                place_rpy = list(self.get_parameter("default_place_rpy").value)
                if not grasp_d:
                    grasp_d = {"xyz": grasp_xyz, "rpy": grasp_rpy}
                if not place_d:
                    place_d = {"xyz": place_xyz, "rpy": place_rpy}
            else:
                return (
                    perr.PARAM_ERROR,
                    "grasp_pose and place_pose are required "
                    "(or set use_default_poses=true in config)",
                )

        if not self._pick_place_client.wait_for_server(timeout_sec=3.0):
            return perr.EXEC_TIMEOUT, "/openarm/pick_place action server unavailable"

        # 构造 goal
        goal = PickPlace.Goal()
        goal.cmd_id = data.get("request_id", "") or str(uuid.uuid4())
        goal.arm = params.get("arm") or data.get("arm") or self._default_arm
        goal.pose_source = pose_source
        goal.target_name = str(params.get("target_name", ""))
        goal.target_index = int(params.get("target_index", 0))
        goal.grasp_pose = _pose_dict_to_msg(grasp_d)
        goal.place_pose = _pose_dict_to_msg(place_d)
        goal.approach_offset_m = float(params.get("approach_offset_m", 0.05))
        goal.retreat_offset_m = float(params.get("retreat_offset_m", 0.05))
        goal.speed_scale = float(params.get("speed_scale", 0.10))
        goal.timeout_s = float(params.get("timeout_s", self._command_timeout_s))

        done_event = threading.Event()
        result_holder: List[Optional[object]] = [None]
        gh_holder: List[Optional[object]] = [None]

        def on_feedback(fb_msg: object) -> None:
            fb = fb_msg.feedback  # type: ignore[attr-defined]
            with self._state_lock:
                self._state = "running"
                self._running = True
                self._ack = "running"
            self.get_logger().debug(
                f"[pick_place] fb status={fb.status} phase={fb.phase} "
                f"progress={fb.progress:.2f}"
            )

        def on_result(res_future: object) -> None:
            result_holder[0] = res_future.result()  # type: ignore[attr-defined]
            with self._goal_handle_lock:
                self._goal_handle = None
            done_event.set()

        def on_goal(future: object) -> None:
            gh = future.result()  # type: ignore[attr-defined]
            gh_holder[0] = gh
            if gh is None or not gh.accepted:
                done_event.set()
                return
            with self._goal_handle_lock:
                self._goal_handle = gh
            gh.get_result_async().add_done_callback(on_result)

        send_fut = self._pick_place_client.send_goal_async(
            goal, feedback_callback=on_feedback
        )
        send_fut.add_done_callback(on_goal)

        timeout = goal.timeout_s + 30.0
        done_event.wait(timeout=timeout)

        if not done_event.is_set():
            return perr.EXEC_TIMEOUT, "pick_place wait timed out"

        gh = gh_holder[0]
        if gh is None or not gh.accepted:  # type: ignore[union-attr]
            return perr.HARDWARE_FAULT, "pick_place goal rejected by skill server"

        wrap = result_holder[0]
        if wrap is None:
            return perr.EXEC_TIMEOUT, "pick_place result future timed out"

        r = wrap.result  # type: ignore[attr-defined]
        skill_code = int(r.result_code)
        proto_code = perr.skill_to_protocol(skill_code)
        msg_str = r.message or ("done" if r.success else "failed")
        self.get_logger().info(
            f"[pick_place] skill_code={skill_code} -> proto_code={proto_code}({perr.name_of(proto_code)}) msg={msg_str}"
        )
        return proto_code, msg_str

    def _exec_home(self, data: dict, params: dict) -> Tuple[int, str]:
        if not self._home_client.wait_for_service(timeout_sec=2.0):
            return perr.EXEC_TIMEOUT, "/openarm/goto_home service unavailable"

        req = GotoHomeSrv.Request()
        req.arm = str(params.get("arm") or data.get("arm") or "both")
        req.speed_scale = float(params.get("speed_scale", 0.30))

        done_event = threading.Event()
        result_holder: List[Optional[object]] = [None]

        def cb(fut: object) -> None:
            result_holder[0] = fut.result()  # type: ignore[attr-defined]
            done_event.set()

        self._home_client.call_async(req).add_done_callback(cb)
        done_event.wait(timeout=30.0)

        r = result_holder[0]
        if r is None:
            return perr.EXEC_TIMEOUT, "goto_home service timed out"
        skill_code = int(r.result_code)  # type: ignore[attr-defined]
        proto_code = perr.skill_to_protocol(skill_code)
        msg_str = r.message or ("done" if r.success else "failed")  # type: ignore[attr-defined]
        return proto_code, msg_str

    def _exec_gripper(self, data: dict, params: dict) -> Tuple[int, str]:
        if not self._gripper_client.wait_for_service(timeout_sec=2.0):
            return perr.EXEC_TIMEOUT, "/openarm/gripper service unavailable"

        req = GripperSrv.Request()
        req.arm = str(params.get("arm") or data.get("arm") or self._default_arm)
        req.action = str(params.get("action", "open"))
        req.position = float(params.get("position", 0.0))
        req.force = float(params.get("force", 0.0))

        done_event = threading.Event()
        result_holder: List[Optional[object]] = [None]

        def cb(fut: object) -> None:
            result_holder[0] = fut.result()  # type: ignore[attr-defined]
            done_event.set()

        self._gripper_client.call_async(req).add_done_callback(cb)
        done_event.wait(timeout=10.0)

        r = result_holder[0]
        if r is None:
            return perr.EXEC_TIMEOUT, "gripper service timed out"
        skill_code = int(r.result_code)  # type: ignore[attr-defined]
        proto_code = perr.skill_to_protocol(skill_code)
        msg_str = r.message or ("done" if r.success else "failed")  # type: ignore[attr-defined]
        return proto_code, msg_str

    def _call_stop_service(self, rid: str) -> Tuple[int, str]:
        """调用 /openarm/stop，1 秒内不可用则直接返回超时码。"""
        if not self._stop_client.wait_for_service(timeout_sec=1.0):
            return perr.EXEC_TIMEOUT, "/openarm/stop service unavailable"

        req = StopSrv.Request()
        req.cmd_id = rid or str(uuid.uuid4())

        done_event = threading.Event()
        result_holder: List[Optional[object]] = [None]

        def cb(fut: object) -> None:
            result_holder[0] = fut.result()  # type: ignore[attr-defined]
            done_event.set()

        self._stop_client.call_async(req).add_done_callback(cb)
        done_event.wait(timeout=3.0)

        r = result_holder[0]
        if r is None:
            return perr.EXEC_TIMEOUT, "stop service timed out"
        if r.success:  # type: ignore[attr-defined]
            return perr.OK, r.message  # type: ignore[attr-defined]
        return perr.HARDWARE_FAULT, r.message  # type: ignore[attr-defined]

    # ======================================================================
    # 状态管理与发布
    # ======================================================================
    def _set_status(
        self,
        rid: str,
        state: str,
        running: bool,
        ack: str,
        error_code: int,
        error_message: str,
    ) -> None:
        with self._state_lock:
            self._last_request_id = rid
            self._state = state
            self._running = running
            self._ack = ack
            self._error_code = error_code
            self._error_message = error_message

    def _publish_status(self) -> None:
        with self._state_lock:
            payload = {
                "type": "manipulator_status",
                "state": self._state,
                "running": self._running,
                "ack": self._ack,
                "error_code": self._error_code,
                "error_message": self._error_message,
                "last_request_id": self._last_request_id,
                "request_id": self._last_request_id,
                "board_id": self._board_id,
                "timestamp": now_ms(),
            }
        out = String()
        out.data = json.dumps(payload)
        self._status_pub.publish(out)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorBoardNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
