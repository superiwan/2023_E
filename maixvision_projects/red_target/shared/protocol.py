"""MaixCAM 与 STM32 共用的 9 字节固定帧协议。"""

from enum import IntEnum


HEADER = b"\xAA\x55"
FRAME_SIZE = 9


class MessageType(IntEnum):
    WAYPOINT = 0x01
    VISION_ERROR = 0x02
    COMPLETE = 0x03
    SPOT_LOST = 0x04
    TRACK_STATUS = 0x05
    START_RESUME = 0x10
    PAUSE = 0x11
    REACQUIRE = 0x12
    SELECT_MODE = 0x13


def _encode_value(value, signed):
    lower = -32768 if signed else 0
    upper = 32767 if signed else 65535
    if not lower <= value <= upper:
        raise ValueError("数据字段超出 16 位范围")
    return int(value).to_bytes(2, "little", signed=signed)


def encode_frame(message_type, index=0, data0=0, data1=0):
    message_type = MessageType(message_type)
    if not 0 <= index <= 255:
        raise ValueError("索引必须位于 0 到 255")

    signed = message_type == MessageType.VISION_ERROR
    packet = HEADER + bytes((message_type, index))
    packet += _encode_value(data0, signed)
    packet += _encode_value(data1, signed)
    return packet + bytes((sum(packet) & 0xFF,))


class FrameParser:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        if data:
            self._buffer.extend(data)

        frames = []
        while True:
            header_index = self._buffer.find(HEADER)
            if header_index < 0:
                if self._buffer[-1:] == HEADER[:1]:
                    self._buffer[:] = HEADER[:1]
                else:
                    self._buffer.clear()
                break
            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < FRAME_SIZE:
                break

            packet = self._buffer[:FRAME_SIZE]
            if (sum(packet[:8]) & 0xFF) != packet[8]:
                del self._buffer[0]
                continue

            try:
                message_type = MessageType(packet[2])
            except ValueError:
                del self._buffer[:FRAME_SIZE]
                continue

            signed = message_type == MessageType.VISION_ERROR
            data0 = int.from_bytes(packet[4:6], "little", signed=signed)
            data1 = int.from_bytes(packet[6:8], "little", signed=signed)
            frames.append((message_type, packet[3], data0, data1))
            del self._buffer[:FRAME_SIZE]
        return frames
