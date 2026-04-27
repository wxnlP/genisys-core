from http.server import HTTPServer, BaseHTTPRequestHandler

class TraceHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        # 读取原始数据（不进行任何解码）
        raw_data = self.rfile.read(content_length)
        
        print(f"\n{'='*40}")
        print(f"收到 POST 请求: {self.path}")
        print(f"Content-Type: {self.headers.get('Content-Type')}")
        print(f"数据长度: {len(raw_data)} bytes")
        print(f"原始数据 (Hex): {raw_data.hex(' ')}") 
        print(f"原始数据 (尝试按字符串读): {raw_data[:100]}") # 打印前100字节看是否有乱码
        print(f"{'='*40}")
        
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"OK")

# 这里的端口要和你 AimRT 配置中发送的目标端口一致
# 比如 URL 是 http://127.0.0.1:50081/... 那么这里就监听 50081
server_address = ('127.0.0.1', 50081)
httpd = HTTPServer(server_address, TraceHandler)
print(f"正在 50081 端口监听 AimRT 数据推送...")
httpd.serve_forever()