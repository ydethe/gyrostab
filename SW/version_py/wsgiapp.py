import random
import json


class IMU (object):
   def __init__(self):
      self.x = 0.
      self.tx = []
      self.ty = []

   def __call__(self, environ, start_response):
      """A barebones WSGI application.
   
      This is a starting point for your own Web framework :)
      """
      pth = environ['PATH_INFO'][1:]
      if pth == '':
         pth = 'index.html'
         
      if pth == 'data.json':
         response_headers = [('Content-Type', 'text/json')]
         self.tx.append(self.x)
         self.ty.append(random.random())
         self.x += 1.
         
         data = {"label":"IMU","data": zip(self.tx,self.ty)}
         res = json.dumps(data)
         
      elif pth == 'flot/examples.css':
         response_headers = [('Content-Type', 'text/css')]
         res = open(pth, 'r')
         
      else:
         response_headers = [('Content-Type', 'text/html')]
         res = open(pth, 'r')
         
      status = '200 OK'
      start_response(status, response_headers)
      return res
    
def test():
    import server
    a = IMU()
    server.main(a)
   
if __name__ == '__main__':
    test()
    
    