import socket
import json
import matplotlib.pyplot as plt

HOST = '0.0.0.0'
PORT = 8002  # Changed to match the port in your request

x = []
t = []
y = []
z = []
i = 0

def handle_http_request(request):
    """Extract JSON payload from HTTP request"""
    parts = request.split('\r\n\r\n', 1)
    if len(parts) > 1:
        return parts[1]  # Return the body
    return None

def handle_client(connection, client_address):
    global i
    buffer = b''
    try:
        print(f"Connected to client at {client_address}")
        while True:
            # Receive data in chunks
            data = connection.recv(4096)
            if not data:
                break
            
            buffer += data
            buffer = buffer.decode("utf-8")
            json_string = buffer.split('\n')[-3]
            data_dict = json.loads(json_string)
            print(data_dict['iot2tangle'][0]['data'][0])
            plt.clf()
            x.append(int(data_dict['iot2tangle'][0]['data'][0]['x']))
            t.append(i)
            i += 1
            plt.scatter(y=x,x=t)
            plt.pause(0.05)
            plt.ioff()
            y.append(int(data_dict['iot2tangle'][0]['data'][0]['y']))
            z.append(int(data_dict['iot2tangle'][0]['data'][0]['z']))
            
            
            # Check if we've received the complete HTTP request
            # if b'\r\n\r\n' in buffer:
            #     # Try to get Content-Length if available
            #     headers, _, body = buffer.partition(b'\r\n\r\n')
            #     headers = headers.decode('utf-8')
                
            #     content_length = 0
            #     for line in headers.split('\r\n'):
            #         if line.lower().startswith('content-length:'):
            #             content_length = int(line.split(':')[1].strip())
                
            #     # Wait for complete body if we know the length
            #     if content_length > 0 and len(body) < content_length:
            #         continue  # Need to receive more data
                
            #     # Process the complete message
            #     decoded_data = buffer.decode('utf-8')
                
            #     json_payload = handle_http_request(decoded_data)
            #     print(json_payload)
            #     if json_payload:
            #         try:
            #             sensor_data = json.loads(json_payload)
            #             print("Received sensor data:")
            #             print(json.dumps(sensor_data, indent=4))
                        
            #             # Process sensor data
            #             for item in sensor_data['iot2tangle']:
            #                 sensor = item['sensor']
            #                 for entry in item['data']:
            #                     print(f"{sensor}: {entry}")
                                
            #             # Send HTTP response
            #             response = (
            #                 "HTTP/1.1 200 OK\r\n"
            #                 "Content-Type: application/json\r\n"
            #                 "Connection: close\r\n"
            #                 "\r\n"
            #                 '{"status": "success"}'
            #             )
            #             connection.sendall(response.encode('utf-8'))
            #             buffer = b''  # Clear buffer for next message
                        
            #         except json.JSONDecodeError as e:
            #             print(f"JSON decode error: {e}")
            #             print(f"Raw payload:\n{json_payload}")
            #         except KeyError as e:
            #             print(f"Missing key in data: {e}")
                
    except ConnectionResetError:
        print("Client forcibly closed the connection")
    finally:
        connection.close()

def start_tcp_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        print(f"Server started, listening on {HOST}:{PORT}")
        server_socket.listen(1)
        
        while True:
            connection, client_address = server_socket.accept()
            handle_client(connection, client_address)

if __name__ == "__main__":
    start_tcp_server()

