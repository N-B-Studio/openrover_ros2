import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(("0.0.0.0",8888))

count=0
start=time.time()

while True:

    data,addr=sock.recvfrom(2048)

    count+=1

    if count%1000==0:

        now=time.time()

        rate=count/(now-start)

        print(rate,"pkt/s",data.decode().strip())