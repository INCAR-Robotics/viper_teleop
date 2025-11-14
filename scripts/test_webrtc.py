import asyncio
from incar.webrtc.webrtc_connection import WebRTCConnection

if __name__ == "__main__":
    rtc = WebRTCConnection().add_channel("test")
    asyncio.run(rtc.start_connection("192.168.200.111", 9997, False))