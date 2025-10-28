import asyncio
import websockets

async def listen():
    uri = "ws://localhost:8000"
    async with websockets.connect(uri) as websocket:
        print(f"Connected to {uri}")
        try:
            async for message in websocket:
                print(f"Received: {message}")
        except websockets.ConnectionClosed:
            print("Disconnected from server")

if __name__ == "__main__":
    asyncio.run(listen())
