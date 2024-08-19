import json
import zmq

# Create a ZMQ context
context = zmq.Context()

# Define the socket type (SUB for subscriber)
socket_type = zmq.SUB

# Create a socket
socket = context.socket(socket_type)

# Subscribe to all topics (replace "" with a specific topic if needed)
socket.subscribe("")

# Connect the socket to the publisher (replace "tcp://localhost:5555" with the publisher's address)
socket.connect("tcp://localhost:5555")

# Loop to receive data
while True:
  # Receive data as a JSON string
  data_string = socket.recv_json()
  
  # Convert JSON string to Python data structure
  data = json.loads(data_string)
  
  # Print the received data
  print("Received data:", data)

# Close the socket (when finished)
socket.close()
context.term()

