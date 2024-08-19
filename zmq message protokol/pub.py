import json
import zmq

# Define the data to be sent
data1 = [{'id': 0.0, 'label': 'person', 'conf': 0.72806877, 'bbox': [257.94, 18.185, 480.94, 314.41]},
         {'id': 63.0, 'label': 'laptop', 'conf': 0.49924716, 'bbox': [204.69, 262.82, 447.66, 475.83]}]
data2 = [{'id': 63.0, 'label': 'laptop', 'conf': 0.68126434, 'bbox': [206.36, 259.49, 445.99, 479.16]},
         {'id': 0.0, 'label': 'person', 'conf': 0.61885846, 'bbox': [259.61, 14.857, 485.93, 324.39]}]

# Create a ZMQ context
context = zmq.Context()

# Define the socket type (PUB for publisher)
socket_type = zmq.PUB

# Create a socket
socket = context.socket(socket_type)

# Bind the socket to a port (replace 5555 with any desired port number)
socket.bind("tcp://localhost:5555")

# Send the data in JSON format
socket.send_json(data1)
socket.send_json(data2)

# Close the socket
socket.close()
context.term()

print("Data sent successfully!")