import websockets
import asyncio
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)  #ignore deprecation warnings

async def listen():
    url = "ws://192.168.0.46:7890" #this is the local machine ip address with the port number
    async with websockets.connect(url) as ws: #connect to the server
        await ws.send("Hello Server!") #Send a greeting message
        # stay alive forever listening for messages
        while True:
            msg = await ws.recv()
            print(msg)

asyncio.get_event_loop().run_until_complete(listen())