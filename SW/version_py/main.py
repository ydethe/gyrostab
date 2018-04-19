import socket
import json
import os
import network


def main():
   
   addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
   
   s = socket.socket()
   s.bind(addr)
   s.listen(1)
   
   print('listening on', addr)
   
   tx = []
   ty = []
   x = 0.
   while True:
      cl, addr = s.accept()
      print('client connected from', addr)
      cl_file = cl.makefile('rwb', 0)
      cmd = cl_file.readline().decode('ascii').split()
      pth = cmd[1][1:]
      if pth == '':
         pth = 'template.html'
         
      while True:
         line = cl_file.readline()
         if not line or line == b'\r\n':
            break
      
      if pth == 'data.json':
         tx.append(x)
         ty.append(random.random())
         x += 1.
         
         data = {'label':'IMU','data': zip(tx,ty)}
         response = json.dumps(data)
      else:
         print("Lecture de " + pth)
         f = open(pth, 'r')
         response = f.read()
         f.close()
   
      b_sent = 0
      b_response = len(response)
      while b_sent != b_response:
         b_sent += cl.send(response[b_sent:])
         
      cl.close()
      
if __name__ == '__main__':
   pass
   # main()
   