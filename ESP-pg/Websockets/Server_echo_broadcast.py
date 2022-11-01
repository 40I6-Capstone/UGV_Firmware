#Server Echo
import websockets
import asyncio
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)  #ignore deprecation warnings

PORT = 7890

print("Started listening on Port " + str(PORT))

connected = set()

#define an asynchronous function to echo back what the client sends
async def echo(websocket, path):
    print("A client has connected")
    connected.add(websocket) #add a copy of the websocket that just connected to the set above - this keeps a list of all clients connoected
    try:
        async for message in websocket:
            print("Received message from client: " + message)
            for conn in connected:
                if conn != websocket: #if conn is different than the one that sent the message then send the message
                    await conn.send("Someone said: " + message)
    except websockets.exceptions.ConnectionClosed as error: #try-catch to handle error messages that appear when a client disconnects
        print("A client disconnected")
        print(error)

#start the websocket server and keep it running forever
start_server = websockets.serve(echo, "localhost", PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()