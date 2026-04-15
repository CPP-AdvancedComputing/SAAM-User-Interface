#!/usr/bin/env python3
import argparse
import os
import socket
import struct
import sys
import time


WEBREPL_REQ_STRUCT = "<2sBBQLH64s"
WEBREPL_PUT_FILE = 1
WEBREPL_GET_FILE = 2
WEBREPL_GET_VER = 3
FRAME_TEXT = 0x81
FRAME_BINARY = 0x82


class WebreplError(RuntimeError):
    pass


class SimpleWebSocket:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b""

    def recv_exactly(self, size):
        chunks = []
        remaining = size
        while remaining > 0:
            data = self.sock.recv(remaining)
            if not data:
                raise WebreplError("unexpected EOF while reading websocket data")
            chunks.append(data)
            remaining -= len(data)
        return b"".join(chunks)

    def write(self, data, opcode=FRAME_BINARY):
        if isinstance(data, str):
            data = data.encode("utf-8")
        length = len(data)
        if length < 126:
            header = struct.pack(">BB", opcode, length)
        else:
            header = struct.pack(">BBH", opcode, 126, length)
        self.sock.sendall(header)
        self.sock.sendall(data)

    def read(self, size, text_ok=False):
        while len(self.buffer) < size:
            frame_head = self.recv_exactly(2)
            opcode, length = struct.unpack(">BB", frame_head)
            if length == 126:
                (length,) = struct.unpack(">H", self.recv_exactly(2))
            if opcode == FRAME_BINARY or (text_ok and opcode == FRAME_TEXT):
                self.buffer += self.recv_exactly(length)
            else:
                _ = self.recv_exactly(length)
        data = self.buffer[:size]
        self.buffer = self.buffer[size:]
        return data


def log(message):
    print(message, flush=True)


def client_handshake(sock, host):
    stream = sock.makefile("rwb", 0)
    request = (
        "GET / HTTP/1.1\r\n"
        f"Host: {host}\r\n"
        "Connection: Upgrade\r\n"
        "Upgrade: websocket\r\n"
        "Sec-WebSocket-Key: sam-pico-webrepl\r\n"
        "\r\n"
    ).encode("utf-8")
    stream.write(request)
    status = stream.readline()
    if b"101" not in status:
        raise WebreplError(f"websocket upgrade failed: {status.decode('utf-8', 'replace').strip()}")
    while True:
        line = stream.readline()
        if line in (b"", b"\r\n"):
            break


def login(ws, password):
    prompt = b""
    while True:
        char = ws.read(1, text_ok=True)
        prompt += char
        if prompt.endswith(b": "):
            break
    ws.write(password.encode("utf-8") + b"\r", opcode=FRAME_TEXT)


def read_resp(ws):
    data = ws.read(4)
    sig, code = struct.unpack("<2sH", data)
    if sig != b"WB":
        raise WebreplError(f"unexpected WebREPL response signature: {sig!r}")
    return code


def get_version(ws):
    request = struct.pack(WEBREPL_REQ_STRUCT, b"WA", WEBREPL_GET_VER, 0, 0, 0, 0, b"")
    ws.write(request)
    version = ws.read(3)
    return struct.unpack("BBB", version)


def put_file(ws, local_file, remote_file):
    file_size = os.stat(local_file).st_size
    remote_name = remote_file.encode("utf-8")
    if len(remote_name) > 64:
        raise WebreplError("remote path is too long for WebREPL (max 64 bytes)")

    record = struct.pack(
        WEBREPL_REQ_STRUCT,
        b"WA",
        WEBREPL_PUT_FILE,
        0,
        0,
        file_size,
        len(remote_name),
        remote_name,
    )

    ws.write(record[:10])
    ws.write(record[10:])
    if read_resp(ws) != 0:
        raise WebreplError("device rejected put-file request")

    sent = 0
    last_report = -1
    with open(local_file, "rb") as handle:
      while True:
            chunk = handle.read(1024)
            if not chunk:
                break
            ws.write(chunk)
            sent += len(chunk)
            pct = int((sent * 100) / file_size) if file_size else 100
            if pct != last_report and (pct == 100 or pct // 10 != last_report // 10):
                log(f"Upload progress {pct}% ({sent}/{file_size} bytes)")
                last_report = pct

    if read_resp(ws) != 0:
        raise WebreplError("device reported failure after upload")


def get_file(ws, local_file, remote_file):
    remote_name = remote_file.encode("utf-8")
    if len(remote_name) > 64:
        raise WebreplError("remote path is too long for WebREPL (max 64 bytes)")

    record = struct.pack(
        WEBREPL_REQ_STRUCT,
        b"WA",
        WEBREPL_GET_FILE,
        0,
        0,
        0,
        len(remote_name),
        remote_name,
    )
    ws.write(record)
    if read_resp(ws) != 0:
        raise WebreplError("device rejected get-file request")

    received = 0
    with open(local_file, "wb") as handle:
        while True:
            ws.write(b"\0")
            (chunk_size,) = struct.unpack("<H", ws.read(2))
            if chunk_size == 0:
                break
            chunk = ws.read(chunk_size)
            handle.write(chunk)
            received += len(chunk)
            if received == len(chunk) or received % 1024 == 0:
                log(f"Download progress {received} bytes")

    log(f"Download complete ({received} bytes)")


def send_reset(ws):
    ws.write("import machine; machine.reset()\r", opcode=FRAME_TEXT)
    time.sleep(0.25)


def connect_webrepl(host, port, password):
    sock = socket.socket()
    sock.settimeout(10)
    addr = socket.getaddrinfo(host, port)[0][4]
    sock.connect(addr)
    client_handshake(sock, host)
    ws = SimpleWebSocket(sock)
    login(ws, password)
    version = get_version(ws)
    log(f"Connected to WebREPL {host}:{port} (version {version[0]}.{version[1]}.{version[2]})")
    return sock, ws


def run_put(args):
    source = os.path.abspath(args.src)
    if not os.path.isfile(source):
        raise WebreplError(f"local file not found: {source}")
    log(f"Uploading {source} -> {args.host}:{args.port}:{args.dst}")
    sock, ws = connect_webrepl(args.host, args.port, args.password)
    try:
        put_file(ws, source, args.dst)
        log("Upload complete")
        if args.reset_after:
            log("Requesting Pico reset")
            send_reset(ws)
            log("Reset command sent")
    finally:
        sock.close()


def run_reset(args):
    log(f"Sending reset command to {args.host}:{args.port}")
    sock, ws = connect_webrepl(args.host, args.port, args.password)
    try:
        send_reset(ws)
        log("Reset command sent")
    finally:
        sock.close()


def run_get(args):
    destination = os.path.abspath(args.dst)
    os.makedirs(os.path.dirname(destination), exist_ok=True)
    log(f"Downloading {args.src} <- {args.host}:{args.port}")
    sock, ws = connect_webrepl(args.host, args.port, args.password)
    try:
        get_file(ws, destination, args.src)
    finally:
        sock.close()


def build_parser():
    parser = argparse.ArgumentParser(description="Minimal WebREPL upload/reset client for Pico W boards.")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common(subparser):
        subparser.add_argument("--host", required=True)
        subparser.add_argument("--port", type=int, default=8266)
        subparser.add_argument("--password", required=True)

    put_parser = sub.add_parser("put", help="Upload a local file to the Pico over WebREPL.")
    add_common(put_parser)
    put_parser.add_argument("--src", required=True)
    put_parser.add_argument("--dst", required=True)
    put_parser.add_argument("--reset-after", action="store_true")

    reset_parser = sub.add_parser("reset", help="Issue machine.reset() over WebREPL.")
    add_common(reset_parser)

    get_parser = sub.add_parser("get", help="Download a remote file from the Pico over WebREPL.")
    add_common(get_parser)
    get_parser.add_argument("--src", required=True)
    get_parser.add_argument("--dst", required=True)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        if args.command == "put":
            run_put(args)
        elif args.command == "get":
            run_get(args)
        else:
            run_reset(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
