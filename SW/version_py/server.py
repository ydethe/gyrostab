# Tested with Python 2.7.9, Linux & Mac OS X
import socket
import sys


class WSGIServer(object):

    address_family = socket.AF_INET
    socket_type = socket.SOCK_STREAM
    request_queue_size = 1

    def __init__(self, server_address):
        # Create a listening socket
        self.listen_socket = listen_socket = socket.socket(
            self.address_family,
            self.socket_type
        )
        # Allow to reuse the same address
        listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind
        listen_socket.bind(server_address)
        # Activate
        listen_socket.listen(self.request_queue_size)
        # Get server host name and port
        host, port = server_address
        # host, port = self.listen_socket.getsockname()[:2]
        self.server_name = host
        # self.server_name = socket.getfqdn(host)
        self.server_port = port
        # Return headers set by Web framework/Web application
        self.headers_set = []

    def set_app(self, application):
        self.application = application

    def serve_forever(self):
        listen_socket = self.listen_socket
        while True:
            # New client connection
            self.client_connection, client_address = listen_socket.accept()
            # Handle one request and close the client connection. Then
            # loop over to wait for another client connection
            self.handle_one_request()

    def handle_one_request(self):
        self.request_data = request_data = self.client_connection.recv(1024)
        # Print formatted request data a la 'curl -v'
        # print(''.join(
        #     '< {line}\n'.format(line=line)
        #     for line in request_data.splitlines()
        # ))

        self.parse_request(request_data)

        # Construct environment dictionary using request data
        env = self.get_environ()

        # It's time to call our application callable and get
        # back a result that will become HTTP response body
        result = self.application(env, self.start_response)

        # Construct a response and send it back to the client
        self.finish_response(result)

    def parse_request(self, text):
        request_line = text.split(b'\n')[0].decode('ascii')
        request_line = request_line.rstrip('\r\n')
        # Break down the request line into components
        (self.request_method,  # GET
         self.path,            # /hello
         self.request_version  # HTTP/1.1
         ) = request_line.split()

    def get_environ(self):
        env = {}
        # The following code snippet does not follow PEP8 conventions
        # but it's formatted the way it is for demonstration purposes
        # to emphasize the required variables and their values
        #
        # Required WSGI variables
        env['wsgi.version']      = (1, 0)
        env['wsgi.url_scheme']   = 'http'
        env['wsgi.input']        = self.request_data
        env['wsgi.errors']       = sys.stderr
        env['wsgi.multithread']  = False
        env['wsgi.multiprocess'] = False
        env['wsgi.run_once']     = False
        # Required CGI variables
        env['REQUEST_METHOD']    = self.request_method    # GET
        env['PATH_INFO']         = self.path              # /hello
        env['SERVER_NAME']       = self.server_name       # localhost
        env['SERVER_PORT']       = str(self.server_port)  # 8888
        return env

    def start_response(self, status, response_headers, exc_info=None):
        # Add necessary server headers
        server_headers = [
            ('Date', 'Tue, 31 Mar 2015 12:54:48 GMT'),
            ('Server', 'WSGIServer 0.2'),
        ]
        self.headers_set = [status, response_headers + server_headers]
        # To adhere to WSGI specification the start_response must return
        # a 'write' callable. We simplicity's sake we'll ignore that detail
        # for now.
        # return self.finish_response
    
    def send(self, data):
        b_tot = len(data)
        b_sent = 0
        while b_sent != b_tot:
            deb = b_sent
            b_sent += self.client_connection.send(data[b_sent:])
            
    def sendfile(self, f, fd_len=1000):
        rd_len = 8000
        while rd_len == fd_len:
            data = f.read(fd_len)
            rd_len = len(data)
            self.send(data)
            
    def finish_response(self, result):
        # try:
        if True:
            status, response_headers = self.headers_set
            response = 'HTTP/1.1 {status}\r\n'.format(status=status)
            for header in response_headers:
                response += '{0}: {1}\r\n'.format(*header)
            response += '\r\n'
            self.send(response)
            
            rd_len = 8000
            if hasattr(result, 'read'):
                self.sendfile(result)
                result.close()
            else:
                self.send(result)
            
        # finally:
        #     self.client_connection.close()


SERVER_ADDRESS = (HOST, PORT) = '', 8080


def make_server(server_address, application):
    server = WSGIServer(server_address)
    server.set_app(application)
    return server

def main(app):
    httpd = make_server(SERVER_ADDRESS, app)
    print('WSGIServer: Serving HTTP on port {port} ...\n'.format(port=PORT))
    httpd.serve_forever()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('Provide a WSGI application object as module:callable')
    app_path = sys.argv[1]
    module, application = app_path.split(':')
    module = __import__(module)
    application = getattr(module, application)
    main(application)
    
    # from server import main;from wsgiapp import app;main(app)
    
    