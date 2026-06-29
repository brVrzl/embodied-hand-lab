import serial
import time


class RH56Hand:
    """
    简单的 RH56 五指灵巧手 Python 驱动
    基于官方 RS485 协议：
      - 帧头：0xEB 0x90
      - 读寄存器命令：0x11
      - 写寄存器命令：0x12
    """

    # 寄存器地址（来自用户手册 2.4）:contentReference[oaicite:2]{index=2}
    REG = {
        "HAND_ID":             1000,
        "REDU_RATIO":          1002,
        "CLEAR_ERROR":         1004,
        "SAVE":                1005,
        "RESET_PARA":          1006,
        "FORCE_CLB":           1009,
        "CURRENT_LIMIT":       1020,
        "DEFAULT_SPEED_SET":   1032,
        "DEFAULT_FORCE_SET":   1044,
        "POS_SET":             1474,
        "ANGLE_SET":           1486,
        "FORCE_SET":           1498,
        "SPEED_SET":           1522,
        "POS_ACT":             1534,
        "ANGLE_ACT":           1546,
        "FORCE_ACT":           1582,
        "CURRENT":             1594,
        "ERROR":               1606,
        "STATUS":              1612,
        "TEMP":                1618,
        "ACTION_SEQ_CHECKDATA1":   2000,
        "ACTION_SEQ_CHECKDATA2":   2001,
        "ACTION_SEQ_STEPNUM":  2002,
        "ACTION_SEQ_STEP0":    2016,
        "ACTION_SEQ_STEP1":    2054,
        "ACTION_SEQ_STEP2":    2092,
        "ACTION_SEQ_STEP3":    2130,
        "ACTION_SEQ_STEP4":    2168,
        "ACTION_SEQ_STEP5":    2206,
        "ACTION_SEQ_STEP6":    2244,
        "ACTION_SEQ_STEP7":    2282,
        "ACTION_SEQ_IDX":      2320,
        "SAVE_ACTION_SEQ":     2321,
        "ACTION_SEQ_RUN":      2322,
        "ACTION_ADJUST_FORCE": 2324,
    }
    ACTION_SEQ_MAX_STEPS = 8
    ACTION_SEQ_STEP_SIZE = 38  # 19short

    def __init__(self, port, baudrate=115200, hand_id=1, timeout=0.05, debug=False):
        """
        :param port: 串口号，比如 '/dev/ttyUSB0' 或 'COM5'
        :param baudrate: 波特率，默认 115200
        :param hand_id: 灵巧手 ID，默认 1
        :param timeout: 串口读超时
        :param debug: 是否打印发送/接收的原始数据
        """
        self.port = port
        self.baudrate = baudrate
        self.hand_id = hand_id
        self.timeout = timeout
        self.debug = debug
        self.ser: serial.Serial | None = None

    # ---------- 串口相关 ----------

    def open(self):
        """打开串口"""
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ---------- 底层帧封装 ----------

    @staticmethod
    def _checksum(data_bytes: list[int]) -> int:
        """累加和低 8 位（从 ID 开始算）"""
        return sum(data_bytes) & 0xFF

    def _ensure_serial(self):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not opened. Call open() first.")

    def _send_frame(self, frame: bytes):
        self._ensure_serial()
        if self.debug:
            print("TX:", frame.hex(" "))
        self.ser.write(frame)
        # 发送后等待设备处理
        time.sleep(0.005)

    @staticmethod
    def _split_frames(buffer: bytes) -> list[bytes]:
        frames = []
        i = 0
        length = len(buffer)
        while i + 5 <= length:
            if buffer[i] != 0x90 or buffer[i + 1] != 0xEB:
                i += 1
                continue
            if i + 4 >= length:
                break
            data_len = buffer[i + 3]
            total = data_len + 5
            if i + total > length:
                break
            frames.append(buffer[i:i + total])
            i += total
        return frames

    def _recv_frames(self, expected_frames=1, timeout=None) -> list[bytes]:
        self._ensure_serial()
        if timeout is None:
            timeout = max(self.timeout * 4, 0.1)
        deadline = time.time() + timeout
        buf = bytearray()
        frames: list[bytes] = []
        while time.time() < deadline:
            chunk = self.ser.read(64)
            if chunk:
                buf.extend(chunk)
                frames = self._split_frames(buf)
                if self.debug and chunk:
                    print("RX:", chunk.hex(" "))
                if len(frames) >= expected_frames:
                    break
            else:
                time.sleep(0.001)
        return frames

    @staticmethod
    def _checksum_bytes(data_bytes: list[int]) -> int:
        return sum(data_bytes) & 0xFF

    @staticmethod
    def _validate_checksum(frame: bytes) -> bool:
        if len(frame) < 6:
            return False
        total = 0
        for b in frame[2:-1]:
            total += b
        return (total & 0xFF) == frame[-1]

    def _build_payload(self, payload: list[int]) -> bytes:
        chk = self._checksum(payload)
        return bytes([0xEB, 0x90] + payload + [chk])

    def _exchange(self, payload: list[int], expected_frames=1, timeout=None) -> list[bytes]:
        frame = self._build_payload(payload)
        self._send_frame(frame)
        return self._recv_frames(expected_frames=expected_frames, timeout=timeout)

    @staticmethod
    def _frame_address(frame: bytes) -> int:
        return frame[5] | (frame[6] << 8)

    # ---------- 寄存器读写 ----------

    def read_register(self, address: int, length: int) -> list[int]:
        """
        读寄存器，返回寄存器的原始字节列表（长度 = length）
        对应用户手册 2.2.1“读灵巧手寄存器的操作”:contentReference[oaicite:4]{index=4}
        """
        data = [
            self.hand_id,
            0x04,              # 数据部分长度
            0x11,              # 读寄存器命令
            address & 0xFF,    # 地址低 8 位
            (address >> 8) & 0xFF,  # 地址高 8 位
            length,            # 要读的长度（字节数）
        ]
        frames = self._exchange(data, expected_frames=1)
        if not frames:
            return []
        frame = frames[0]
        if (frame[0], frame[1], frame[2]) != (0x90, 0xEB, self.hand_id):
            return []
        if frame[4] != 0x11 or self._frame_address(frame) != address:
            return []
        reg_len = (frame[3] & 0xFF) - 3
        if reg_len != length or not self._validate_checksum(frame):
            return []
        data_bytes = frame[7:7 + reg_len]
        return list(data_bytes)

    def write_register(self, address: int, data_bytes: list[int], timeout=None) -> bool:
        """
        写寄存器，对应用户手册 2.2.2“写灵巧手寄存器的操作”:contentReference[oaicite:5]{index=5}
        :param address: 起始寄存器地址
        :param data_bytes: 要写入的原始字节列表
        :return: 是否收到正常应答（简单检查）
        """
        length = len(data_bytes)
        data = [
            self.hand_id,
            length + 3,        # 数据部分长度 = 数据字节数 + 3
            0x12,              # 写寄存器
            address & 0xFF,
            (address >> 8) & 0xFF,
        ] + data_bytes
        frames = self._exchange(data, expected_frames=1, timeout=timeout)
        if not frames:
            return False
        frame = frames[0]
        if (frame[0], frame[1], frame[2]) != (0x90, 0xEB, self.hand_id):
            return False
        if frame[4] != 0x12 or self._frame_address(frame) != address:
            return False
        if frame[3] != 0x04 or frame[7] != 0x01:
            return False
        return self._validate_checksum(frame)

    def _wait_status_frame(self, expected_addr: int | None = None, timeout=1.0) -> tuple[bool, int]:
        frames = self._recv_frames(expected_frames=1, timeout=timeout)
        if not frames:
            return False, -1
        frame = frames[0]
        if frame[0] != 0x90 or frame[1] != 0xEB or frame[2] != self.hand_id:
            return False, -1
        if frame[4] != 0x11:
            return False, -1
        if expected_addr is not None and self._frame_address(frame) != expected_addr:
            return False, -1
        if not self._validate_checksum(frame):
            return False, -1
        return True, frame[7]

    # ---------- 一些工具函数 ----------

    @staticmethod
    def _u16_list_from_bytes(data: list[int], count: int) -> list[int]:
        """将偶数长度的 byte 列表转成 count 个 little-endian uint16/int16"""
        if len(data) < 2 * count:
            return []
        vals = []
        for i in range(count):
            lo = data[2 * i] & 0xFF
            hi = data[2 * i + 1]
            vals.append(lo + (hi << 8))
        return vals

    @staticmethod
    def _u16_list_to_bytes(values: list[int]) -> list[int]:
        """将 int16 列表转成 little-endian 字节列表"""
        out = []
        for v in values:
            if v < 0:
                v &= 0xFFFF  # -1 -> 0xFFFF
            out.append(v & 0xFF)
            out.append((v >> 8) & 0xFF)
        return out

    @staticmethod
    def _validate_range(values: list[int], min_val: int, max_val: int, allow_neg1=False):
        for v in values:
            if allow_neg1 and v == -1:
                continue
            if v < min_val or v > max_val:
                raise ValueError(f"数值 {v} 超出范围 [{min_val}, {max_val}]")

    # ---------- 高层封装：六个自由度读写 ----------

    def set_speeds(self, speeds: list[int]) -> bool:
        """
        设置 6 个自由度速度（0~1000，-1 表示不动作）
        对应 SPEED_SET(m)，地址 1522:contentReference[oaicite:6]{index=6}
        """
        if len(speeds) != 6:
            raise ValueError("speeds 必须是长度为 6 的 list")
        self._validate_range(speeds, 0, 1000, allow_neg1=False)
        data = self._u16_list_to_bytes(speeds)
        return self.write_register(self.REG["SPEED_SET"], data)

    def set_forces(self, forces: list[int]) -> bool:
        """
        设置 6 个自由度力控阈值（单位大致为 g，0~1000）:contentReference[oaicite:7]{index=7}
        对应 FORCE_SET(m)
        """
        if len(forces) != 6:
            raise ValueError("forces 必须是长度为 6 的 list")
        self._validate_range(forces, 0, 1000, allow_neg1=False)
        data = self._u16_list_to_bytes(forces)
        return self.write_register(self.REG["FORCE_SET"], data)

    def set_angles(self, angles: list[int]) -> bool:
        """
        设置 6 个自由度角度（0~1000，-1 表示该指保持不动）
        对应 ANGLE_SET(m)，地址 1486:contentReference[oaicite:8]{index=8}
        """
        if len(angles) != 6:
            raise ValueError("angles 必须是长度为 6 的 list")
        self._validate_range(angles, 0, 1000, allow_neg1=True)
        data = self._u16_list_to_bytes(angles)
        return self.write_register(self.REG["ANGLE_SET"], data)

    def get_angles(self) -> list[int]:
        """
        读取当前 6 个自由度角度实际值 ANGLE_ACT(m):contentReference[oaicite:9]{index=9}
        返回 6 个 int
        """
        raw = self.read_register(self.REG["ANGLE_ACT"], 12)
        return self._u16_list_from_bytes(raw, 6)

    def get_forces(self) -> list[int]:
        """
        读取 6 个手指的实际受力 FORCE_ACT(m)，单位 g:contentReference[oaicite:10]{index=10}
        """
        raw = self.read_register(self.REG["FORCE_ACT"], 12)
        return self._u16_list_from_bytes(raw, 6)

    def get_temps(self) -> list[int]:
        """
        读取 6 个电缸温度 TEMP(m)，单位 ℃:contentReference[oaicite:11]{index=11}
        """
        raw = self.read_register(self.REG["TEMP"], 6)
        return raw  # 每个就是 1 byte

    def get_status(self) -> list[int]:
        """
        读取 6 个自由度状态信息 STATUS(m):contentReference[oaicite:12]{index=12}
        """
        raw = self.read_register(self.REG["STATUS"], 6)
        return raw

    def get_errors(self) -> list[int]:
        """
        读取 6 个自由度故障码 ERROR(m):contentReference[oaicite:13]{index=13}
        """
        raw = self.read_register(self.REG["ERROR"], 6)
        return raw

    def get_currents(self) -> list[int]:
        """读取 6 个自由度的实际电流 CURRENT(m)，单位 mA"""
        raw = self.read_register(self.REG["CURRENT"], 12)
        return self._u16_list_from_bytes(raw, 6)

    def get_positions(self) -> list[int]:
        """读取 6 个自由度的电缸位置 POS_ACT(m)"""
        raw = self.read_register(self.REG["POS_ACT"], 12)
        return self._u16_list_from_bytes(raw, 6)

    # ---------- 一些常用控制 ----------

    def clear_error(self) -> bool:
        """清除错误：向 CLEAR_ERROR 写入 1:contentReference[oaicite:14]{index=14}"""
        return self.write_register(self.REG["CLEAR_ERROR"], [1])

    def save_params(self) -> bool:
        """保存参数到 Flash：向 SAVE 写入 1:contentReference[oaicite:15]{index=15}"""
        if not self.write_register(self.REG["SAVE"], [1]):
            return False
        ok, status = self._wait_status_frame(self.REG["SAVE"], timeout=1.5)
        return ok and status == 0

    def run_action_sequence(self, index: int) -> bool:
        """
        运行动作序列：
          1) ACTION_SEQ_IDX = index
          2) ACTION_SEQ_RUN = 1
        """
        ok1 = self.write_register(self.REG["ACTION_SEQ_IDX"], [index & 0xFF])
        time.sleep(0.05)
        ok2 = self.write_register(self.REG["ACTION_SEQ_RUN"], [1])
        return ok1 and ok2

    def force_calibrate(self) -> bool:
        """启动力传感器校准流程（约 6 秒）"""
        if not self.write_register(self.REG["FORCE_CLB"], [1]):
            return False
        ok, _ = self._wait_status_frame(timeout=6.5)
        return ok

    def set_hand_id(self, new_id: int, persist=False) -> bool:
        """修改 HAND_ID，可选是否立即保存"""
        if not (1 <= new_id <= 254):
            raise ValueError("ID 范围应为 1-254")
        if not self.write_register(self.REG["HAND_ID"], [new_id & 0xFF]):
            return False
        self.hand_id = new_id
        return self.save_params() if persist else True

    def set_baudrate(self, redu_ratio: int, persist=False) -> bool:
        """设置波特率：0=115200,1=57600,2=19200"""
        if redu_ratio not in (0, 1, 2):
            raise ValueError("波特率枚举仅支持 0/1/2")
        ok = self.write_register(self.REG["REDU_RATIO"], [redu_ratio])
        if not ok:
            return False
        return self.save_params() if persist else True

    def reset_factory(self) -> bool:
        """写 RESET_PARA=1，恢复出厂设置"""
        return self.write_register(self.REG["RESET_PARA"], [1])

    def set_current_limits(self, limits: list[int]) -> bool:
        """设置 CURRENT_LIMIT(m)，单位 mA"""
        if len(limits) != 6:
            raise ValueError("limits 必须是长度为 6 的 list")
        self._validate_range(limits, 0, 1500, allow_neg1=False)
        data = self._u16_list_to_bytes(limits)
        return self.write_register(self.REG["CURRENT_LIMIT"], data)

    def set_default_speeds(self, speeds: list[int]) -> bool:
        """设置 DEFAULT_SPEED_SET(m)"""
        if len(speeds) != 6:
            raise ValueError("speeds 必须是长度为 6 的 list")
        limits = [1000, 1000, 1000, 1000, 1500, 1500]
            for i, v in enumerate(forces):
                max_val = limits[i]
                if not (0 <= v <= max_val):
                    raise ValueError(f"通道 {i} 值 {v} 超出范围 [0, {max_val}]")
        data = self._u16_list_to_bytes(speeds)
        return self.write_register(self.REG["DEFAULT_SPEED_SET"], data)

    def set_default_forces(self, forces: list[int]) -> bool:
        """设置 DEFAULT_FORCE_SET(m)"""
        if len(forces) != 6:
            raise ValueError("forces 必须是长度为 6 的 list")
        self._validate_range(forces, 0, 1500, allow_neg1=False)
        data = self._u16_list_to_bytes(forces)
        return self.write_register(self.REG["DEFAULT_FORCE_SET"], data)

    def set_positions(self, positions: list[int]) -> bool:
        """直接设置 POS_SET(m)，范围 0-2000，-1 表示不动作"""
        if len(positions) != 6:
            raise ValueError("positions 必须是长度为 6 的 list")
        self._validate_range(positions, 0, 2000, allow_neg1=True)
        data = self._u16_list_to_bytes(positions)
        return self.write_register(self.REG["POS_SET"], data)

    def save_action_sequence(self, index: int, steps: list[list[int]]) -> bool:
        """
        将动作序列写入 Flash：
          1) ACTION_SEQ_IDX = index
          2) ACTION_SEQ_STEPNUM = len(steps)
          3) ACTION_SEQ_STEP(i) = 19short
          4) CHECKDATA1/2 = 0x90/0xEB
          5) SAVE_ACTION_SEQ = 1
        """
        if not (0 <= index <= 39):
            raise ValueError("动作序列索引范围 0-39")
        if len(steps) > self.ACTION_SEQ_MAX_STEPS:
            raise ValueError("最多 8 个动作步骤")
        if not self.write_register(self.REG["ACTION_SEQ_IDX"], [index & 0xFF]):
            return False
        if not self.write_register(self.REG["ACTION_SEQ_STEPNUM"], [len(steps) & 0xFF]):
            return False
        for i, step in enumerate(steps):
            if len(step) != 19:
                raise ValueError("每个步骤需要 19 个 short")
            data = self._u16_list_to_bytes(step)
            addr = self.REG["ACTION_SEQ_STEP0"] + i * self.ACTION_SEQ_STEP_SIZE
            if not self.write_register(addr, data):
                return False
        if not self.write_register(self.REG["ACTION_SEQ_CHECK1"], [0x90]):
            return False
        if not self.write_register(self.REG["ACTION_SEQ_CHECK2"], [0xEB]):
            return False
        return self.write_register(self.REG["SAVE_ACTION_SEQ"], [1], timeout=0.5)

    # 一些快捷姿态（可以根据自己喜好改）

    def open_hand(self) -> bool:
        """张开手指（实机：角度设为 1000）"""
        return self.set_angles([1000, 1000, 1000, 1000, 1000, 1000])

    def close_hand(self) -> bool:
        """握拳（实机：角度设为 0）"""
        return self.set_angles([0, 0, 0, 0, 0, 0])

        # ---------- 预定义手势（已按 0=握拳, 1000=张开 修正） ----------

    def gesture_pinch(self) -> bool:
        """
        捏取（拇指 + 食指），其余三指半弯，做一个“轻捏”的姿势
        """
        angles = [
            400,  # 小拇指：半弯（原来是 600 -> 1000-600）
            400,  # 无名指：半弯
            500,  # 中指：稍弯（500 -> 500）
            200,  # 食指：较大弯曲（800 -> 200）
            200,  # 大拇指弯曲
            500,  # 大拇指旋转
        ]
        return self.set_angles(angles)

    def gesture_three_finger_grasp(self) -> bool:
        """
        三指抓（拇指 + 食指 + 中指），适合抓细长物体
        """
        angles = [
            100,  # 小拇指（900 -> 100）
            100,  # 无名指
            200,  # 中指（800 -> 200）
            200,  # 食指
            200,  # 大拇指弯曲
            450,  # 大拇指旋转（550 -> 450）
        ]
        return self.set_angles(angles)

    def gesture_ok(self) -> bool:
        """
        OK 手势：拇指和食指形成环，其它手指微弯
        """
        angles = [
            300,  # 小拇指（700 -> 300）
            300,  # 无名指
            500,  # 中指
            200,  # 食指
            200,  # 大拇指弯曲
            400,  # 大拇指旋转（600 -> 400）
        ]
        return self.set_angles(angles)

    def gesture_thumbs_up(self) -> bool:
        """
        竖大拇指：四指握拳，大拇指伸出
        """
        angles = [
            0,    # 小拇指：握拳（1000 -> 0）
            0,    # 无名指
            0,    # 中指
            0,    # 食指
            800,  # 大拇指弯曲（200 -> 800，接近伸直）
            100,  # 大拇指旋转（900 -> 100，转到“竖起”方向，后面你可以微调）
        ]
        return self.set_angles(angles)

    def gesture_victory(self) -> bool:
        """
        V 手势：食指 + 中指伸直，其余收拢
        """
        angles = [
            0,     # 小拇指：弯曲（1000 -> 0）
            0,     # 无名指：弯曲
            1000,  # 中指：伸直（0 -> 1000）
            1000,  # 食指：伸直
            600,   # 大拇指弯曲（400 -> 600，自然一点）
            300,   # 大拇指旋转（700 -> 300）
        ]
        return self.set_angles(angles)
