import sim_adcs
import socket
import numpy as np

# client
host = socket.gethostname()  # as both code is running on same pc
port = 5000  # socket server port number

client_socket = socket.socket()  # instantiate
client_socket.connect((host, port))  # connect to the server

data = client_socket.recv(1024).decode()  # receive response
print(data)
i = 0

adcs = sim_adcs.sim_adcs()
first = True

B1 = np.zeros(3)
B2 = np.zeros(3)
t1 = 0
t2 = 0

while True:
    data1 = client_socket.recv(1024).decode()
    if (data1 == 'stop'): break
    data1 = data1.split(",")
    
    print(data1)
    
    B1 = np.array([float(data1[0]), float(data1[1]), float(data1[2])])
    t1 = float(data1[3])
    
    if (first == False):
        adcs.update_Bdot(B2, B1, t2, t1)
        mtorqr = adcs.calc_M()
        client_socket.send((str(mtorqr[0]) + "," + str(mtorqr[1]) + "," + str(mtorqr[2])).encode())
        
    first = False
    data2 = client_socket.recv(1024).decode()
    if (data2 == 'stop'): break
    data2 = data2.split(",")
    
    print(data2)
    
    B2 = np.array([float(data2[0]), float(data2[1]), float(data2[2])])
    
    t2 = float(data2[3])
    
    adcs.update_Bdot(B1, B2, t1, t2)
    mtorqr = adcs.calc_M()
    print(mtorqr)
    
    client_socket.send((str(mtorqr[0]) + "," + str(mtorqr[1]) + "," + str(mtorqr[2])).encode())
    
    print("sent")
    
    i+=1
    
data1 = client_socket.recv(1024).decode()
    

print('No more data coming')  # show in terminal
print(str(i) + " iterations run")

client_socket.close()  # close the connection
    