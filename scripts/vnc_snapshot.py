#!/usr/bin/env python3
"""Save a PNG of what a VNC viewer (e.g. noVNC in the browser) would show.

Talks RFB 3.8 to an x11vnc started with -nopw (security type "None"), requests
one full raw framebuffer update, and writes it as PNG. Standard library only.

    scripts/vnc_snapshot.py out.png [--port 5900] [--host 127.0.0.1]

Prints the image size and a non-background pixel fraction, a rough signal
that something other than an empty desktop is on screen.
"""
import argparse
import socket
import struct
import sys
import zlib


def recv_exact(s, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('VNC server closed the connection')
        buf += chunk
    return bytes(buf)


def snapshot(host, port, timeout):
    s = socket.create_connection((host, port), timeout=timeout)
    version = recv_exact(s, 12)
    if not version.startswith(b'RFB '):
        raise RuntimeError(f'not an RFB server: {version!r}')
    s.sendall(b'RFB 003.008\n')
    n = recv_exact(s, 1)[0]
    if n == 0:
        reason_len = struct.unpack('>I', recv_exact(s, 4))[0]
        raise RuntimeError(recv_exact(s, reason_len).decode(errors='replace'))
    types = recv_exact(s, n)
    if 1 not in types:
        raise RuntimeError(f'server requires authentication (types {list(types)})')
    s.sendall(bytes([1]))
    if struct.unpack('>I', recv_exact(s, 4))[0] != 0:
        raise RuntimeError('security handshake failed')
    s.sendall(bytes([1]))                              # ClientInit: shared
    width, height = struct.unpack('>HH', recv_exact(s, 4))
    recv_exact(s, 16)                                  # server pixel format
    recv_exact(s, struct.unpack('>I', recv_exact(s, 4))[0])  # desktop name
    # SetPixelFormat: 32 bpp, depth 24, little endian, true colour, RGB shifts 16/8/0.
    s.sendall(struct.pack('>BxxxBBBBHHHBBBxxx', 0, 32, 24, 0, 1, 255, 255, 255, 16, 8, 0))
    s.sendall(struct.pack('>BxHi', 2, 1, 0))           # SetEncodings: Raw only
    s.sendall(struct.pack('>BBHHHH', 3, 0, 0, 0, width, height))  # full, non-incremental
    fb = bytearray(width * height * 4)
    while True:
        msg = recv_exact(s, 1)[0]
        if msg == 0:
            break
        if msg == 2:                                   # Bell
            continue
        if msg == 1:                                   # SetColourMapEntries
            _, count = struct.unpack('>xHH', recv_exact(s, 5))
            recv_exact(s, count * 6)
            continue
        if msg == 3:                                   # ServerCutText
            recv_exact(s, 3)
            recv_exact(s, struct.unpack('>I', recv_exact(s, 4))[0])
            continue
        raise RuntimeError(f'unexpected server message {msg}')
    nrects = struct.unpack('>xH', recv_exact(s, 3))[0]
    for _ in range(nrects):
        x, y, w, h, enc = struct.unpack('>HHHHi', recv_exact(s, 12))
        if enc != 0:
            raise RuntimeError(f'unexpected encoding {enc}')
        data = recv_exact(s, w * h * 4)
        for row in range(h):
            off = ((y + row) * width + x) * 4
            fb[off:off + w * 4] = data[row * w * 4:(row + 1) * w * 4]
    s.close()
    return width, height, fb


def write_png(path, width, height, fb):
    rows = bytearray()
    for y in range(height):
        rows.append(0)
        line = fb[y * width * 4:(y + 1) * width * 4]
        for x in range(width):
            b, g, r = line[x * 4], line[x * 4 + 1], line[x * 4 + 2]   # little endian BGRX
            rows += bytes((r, g, b))

    def chunk(tag, body):
        return (struct.pack('>I', len(body)) + tag + body
                + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff))

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(bytes(rows), 6)))
        f.write(chunk(b'IEND', b''))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('out')
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=5900)
    p.add_argument('--timeout', type=float, default=10.0)
    a = p.parse_args()
    try:
        width, height, fb = snapshot(a.host, a.port, a.timeout)
    except (OSError, RuntimeError) as e:
        print(f'FAIL {e}')
        return 1
    write_png(a.out, width, height, fb)
    corner = bytes(fb[0:3])
    distinct = sum(1 for i in range(0, len(fb), 4 * 97) if bytes(fb[i:i + 3]) != corner)
    frac = distinct / (len(fb) // (4 * 97))
    print(f'wrote {a.out}: {width}x{height}, non-background sample fraction {frac:.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
