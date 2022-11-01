import asyncio
import websockets

# create handler for each connection
async def handler(websocket, path):
    data = await websocket.recv()
    reply = f"Data recieved as:  {data}!"
    await websocket.send(reply)

start_server = websockets.serve(handler, "localhost", 8000)

loop = asyncio.get_event_loop()
loop.run_until_complete(start_server)
loop.run_forever()
