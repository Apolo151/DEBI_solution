import socket
import cv2
import numpy as np

global ip_address, port, server_socket, check, img

def setup_connection(Aip_address="0.0.0.0", Aport=1234):
    global ip_address, port, img
    # Define the local address and port to bind to
    # Define the IP address and port to listen on
    ip_address = Aip_address  # Listen on all available network interfaces
    port = Aport  # Use the same port number as the sender

def get_image():
    global ip_address, port, server_socket, check
    check=False
    # Create a socket to receive data
    server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server_socket.bind((ip_address, port))
    try:
        # Receive the number of frames sent by the sender
        n_frames, _ = server_socket.recvfrom(1024)
        n_frames = int(n_frames.decode())
        print(n_frames)
        #if(n_frames < 43) : continue

        # Receive all the frames sent by the sender
        frames = []
        for i in range(n_frames):
            frame, _ = server_socket.recvfrom(1024)
            frames.append(frame)

        # Concatenate the received frames into a single byte array
        img_bytes = b''.join(frames)

        # Decode the byte array into an image using OpenCV
        
        img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
        # Show the image
        cv2.imshow("Received Image", img)

        # Check for a key press and terminate if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            check=True
    except:
        pass

def close_connection():
    global server_socket
    #Close the socket
    server_socket.close()
    cv2.destroyAllWindows()
        
if __name__=="__main__":
    
    # Define the local address and port to bind to
    # Define the IP address and port to listen on
    ip_address = "0.0.0.0"  # Listen on all available network interfaces
    port = 1234  # Use the same port number as the sender
    while True:
        get_image()
        if check:
            break
    #Close the socket
    server_socket.close()
    cv2.destroyAllWindows()
