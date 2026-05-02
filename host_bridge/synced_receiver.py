#!/usr/bin/env python3
"""
Приёмник: получает видео и данные, сопоставляет по timestamp.
"""
import asyncio
import json
import struct
import websockets

latest_data = {}
latest_frame = None
data_ts = 0.0
frame_ts = 0.0


async def video_reader():
    global latest_frame, frame_ts
    async with websockets.connect('ws://localhost:9091') as ws:
        while True:
            raw = await ws.recv()
            ts = struct.unpack('d', raw[:8])[0]
            jpeg = raw[8:]
            frame_ts = ts
            latest_frame = jpeg


async def data_reader():
    global latest_data, data_ts
    async with websockets.connect('ws://localhost:9090') as ws:
        while True:
            msg = await ws.recv()
            packet = json.loads(msg)
            data_ts = packet['ts']
            latest_data = packet


async def synced_printer():
    while True:
        if latest_frame and latest_data:
            delta = abs(frame_ts - data_ts)
            status = "OK" if delta < 0.1 else f"dT={delta:.2f}s"
            print(f"[{status}] depth={latest_data.get('depth','?')} "
                  f"yaw={latest_data.get('yaw','?')} "
                  f"frame={len(latest_frame)}B")
        await asyncio.sleep(0.1)


async def main():
    await asyncio.gather(
        video_reader(),
        data_reader(),
        synced_printer(),
    )


if __name__ == '__main__':
    asyncio.run(main())
