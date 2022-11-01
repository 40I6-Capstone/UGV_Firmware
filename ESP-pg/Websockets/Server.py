#Server Echo
import websockets
import asyncio
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)  #ignore deprecation warnings

PORT = 7890

print("Started listening on Port " + str(PORT))

#define an asynchronous function to echo back what the client sends
async def echo(websocket, path):
    print("A client has connected")
    try:
        async for message in websocket:
            print("Received message from client: " + message)
            await websocket.send("Response " + message)
    except websockets.exceptions.ConnectionClosed as error: #try-catch to handle error messages that appear when a client disconnects
        print("A client disconnected")
        print(error)

#start the websocket server and keep it running forever
start_server = websockets.serve(echo, "192.168.0.46", PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()