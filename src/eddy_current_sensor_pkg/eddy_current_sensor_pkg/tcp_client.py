import socket


def create_tcp_client():
    """
    创建并返回TCP客户端socket
    """
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client.bind(('192.168.1.20', 0))  # <-- 加这一行
    client.settimeout(5)
    return client


def connect_tcp(client, host="192.168.1.11", port=8080):
    """
    连接传感器
    """
    try:
        client.connect((host, port))
        print(f"TCP connected to {host}:{port}")
        print(f"Socket FD: {client.fileno()}")
    except Exception as e:
        print(f"TCP connection failed: {e}")
        raise