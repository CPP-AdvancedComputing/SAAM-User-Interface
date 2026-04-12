#!/usr/bin/env python3
from __future__ import annotations

import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class NoCacheStaticHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the SAM UI without browser caching.")
    parser.add_argument("--bind", default="0.0.0.0", help="Address to bind to.")
    parser.add_argument("--port", type=int, default=8080, help="TCP port to listen on.")
    parser.add_argument(
        "--directory",
        default=str(Path(__file__).resolve().parent / "dist"),
        help="Directory to serve.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    directory = str(Path(args.directory).resolve())
    handler = functools.partial(NoCacheStaticHandler, directory=directory)
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    server.serve_forever()


if __name__ == "__main__":
    main()
