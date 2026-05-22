"""协议错误码定义（MANIPULATOR_PROTOCOL §9）及与 openarm_skills 内部码的映射。

协议公共码（板卡输出到 /robot_arm/status）：
  0     成功
  1001  参数错误（字段缺失 / 非法）
  1002  命令不支持
  1003  机械臂忙（不可受理新命令）
  1004  安全互锁触发（control_enabled=False）
  2001  执行超时
  2002  硬件故障（规划失败、执行失败等）

openarm_skills 内部码（来自 error_codes.py）：
  0     OK
  1001  PLAN_FAILED
  1002  ACTION_TIMEOUT
  1003  GRIP_NOT_HELD
  1004  EXECUTE_FAILED
  2001  CAMERA_NO_CLOUD
  2002  CAMERA_NO_TARGET
  2003  CAMERA_OUT_OF_RANGE
  2004  PERCEPTION_TIMEOUT
  3001  BAD_REQUEST
  3002  UNSUPPORTED_CMD
  9001  STOPPED_BY_USER
  9002  INTERNAL_ERROR
"""

# ---- 协议错误码 -----------------------------------------------------------
OK = 0
PARAM_ERROR = 1001        # 字段缺失 / JSON 校验失败
CMD_NOT_SUPPORTED = 1002  # 未知命令字
ARM_BUSY = 1003           # 当前正在执行其他命令
SAFETY_INTERLOCK = 1004   # control_enabled=False 时触发
EXEC_TIMEOUT = 2001       # Action / 服务调用超时
HARDWARE_FAULT = 2002     # 规划失败 / 执行失败 / 硬件异常

_CODE_TO_NAME = {
    OK: "OK",
    PARAM_ERROR: "PARAM_ERROR",
    CMD_NOT_SUPPORTED: "CMD_NOT_SUPPORTED",
    ARM_BUSY: "ARM_BUSY",
    SAFETY_INTERLOCK: "SAFETY_INTERLOCK",
    EXEC_TIMEOUT: "EXEC_TIMEOUT",
    HARDWARE_FAULT: "HARDWARE_FAULT",
}

# ---- openarm_skills 内部码 -> 协议码 映射 --------------------------------
# 注意：skills 和协议均有 1001/1002/1003/1004，但含义不同，必须通过此表转换。
_SKILL_TO_PROTOCOL: dict[int, int] = {
    0:    OK,
    1001: HARDWARE_FAULT,    # PLAN_FAILED         -> 硬件/规划故障
    1002: EXEC_TIMEOUT,      # ACTION_TIMEOUT      -> 执行超时
    1003: HARDWARE_FAULT,    # GRIP_NOT_HELD       -> 硬件故障（抓取空载）
    1004: HARDWARE_FAULT,    # EXECUTE_FAILED      -> 硬件故障
    2001: EXEC_TIMEOUT,      # CAMERA_NO_CLOUD     -> 按超时处理（感知不可用）
    2002: HARDWARE_FAULT,    # CAMERA_NO_TARGET    -> 硬件/感知故障
    2003: PARAM_ERROR,       # CAMERA_OUT_OF_RANGE -> 位姿参数超出工作空间
    2004: EXEC_TIMEOUT,      # PERCEPTION_TIMEOUT  -> 超时
    3001: PARAM_ERROR,       # BAD_REQUEST         -> 参数错误
    3002: CMD_NOT_SUPPORTED, # UNSUPPORTED_CMD     -> 命令不支持
    9001: OK,                # STOPPED_BY_USER     -> stop 成功，返回 OK
    9002: HARDWARE_FAULT,    # INTERNAL_ERROR      -> 硬件/内部故障
}


def skill_to_protocol(skill_code: int) -> int:
    """将 openarm_skills 内部错误码转为协议错误码。未知码映射为 HARDWARE_FAULT。"""
    return _SKILL_TO_PROTOCOL.get(skill_code, HARDWARE_FAULT)


def name_of(code: int) -> str:
    """返回协议错误码的名称字符串。"""
    return _CODE_TO_NAME.get(code, f"UNKNOWN_{code}")
