"""Bingda voice-box protocol used by the vehicle's original code.

The device has fixed phrases. A successful write means only that the command
was sent to the serial device; it does not prove the words were audible.
"""
import time

HANDSHAKE = bytes((0xAA, 0x55, 0xFF, 0x55, 0xAA))
STOP_PROMPT = bytes((0xAA, 0x55, 0x05, 0x55, 0xAA))


def firmware_reply(data):
    for i in range(max(0, len(data) - 5)):
        frame = data[i:i+6]
        if frame[:3] == b'\xaa\x55\x01' and frame[4:] == b'\x55\xaa':
            return frame[3] / 10.0
    return None


class VoiceBox:
    def __init__(self, serial_port, clock=time.monotonic):
        self.port, self.clock = serial_port, clock
    def handshake(self, timeout=4):
        self.port.reset_input_buffer()
        if self.port.write(HANDSHAKE) != len(HANDSHAKE): raise IOError('incomplete voice handshake')
        self.port.flush()
        data, deadline = bytearray(), self.clock() + timeout
        while self.clock() < deadline:
            data.extend(self.port.read(64))
            version = firmware_reply(data)
            if version is not None: return version
            if len(data) > 1024: del data[:-6]
        raise IOError('voice device did not answer handshake')
    def play_stop(self):
        if self.port.write(STOP_PROMPT) != len(STOP_PROMPT): raise IOError('incomplete voice command')
        self.port.flush()
