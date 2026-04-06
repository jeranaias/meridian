#!/usr/bin/env python3
"""
Meridian firmware packaging tool.

Produces .apj files compatible with ArduPilot's GCS upload tools
(Mission Planner, QGroundControl).

APJ format: JSON with base64(zlib(binary)) image + metadata.
"""

import json
import base64
import zlib
import hashlib
import sys
import os
from datetime import datetime

def pack_firmware(bin_path: str, board_id: int, board_name: str = "Meridian") -> dict:
    """Pack a binary firmware file into APJ format."""
    with open(bin_path, 'rb') as f:
        firmware = f.read()

    # Compress with zlib
    compressed = zlib.compress(firmware, 9)

    # Base64 encode
    image = base64.b64encode(compressed).decode('ascii')

    # SHA256 of original binary
    sha256 = hashlib.sha256(firmware).hexdigest()

    # Git hash (if in a git repo)
    git_sha = "unknown"
    try:
        import subprocess
        result = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, cwd=os.path.dirname(bin_path))
        if result.returncode == 0:
            git_sha = result.stdout.strip()
    except Exception:
        pass

    apj = {
        "board_id": board_id,
        "magic": "APJFWv1",
        "description": f"Meridian firmware for {board_name}",
        "image": image,
        "image_size": len(firmware),
        "flash_total": len(firmware),
        "flash_free": 0,
        "git_identity": git_sha,
        "board_revision": 0,
        "USBID": "0x1209/0x5741",  # default Meridian USB ID
        "build_type": "Meridian",
        "summary": f"Meridian {board_name}",
        "vehicletype": "Copter",
        "manufacturer": "Meridian",
        "brand_name": board_name,
        "sha256": sha256,
        "build_time": datetime.utcnow().isoformat() + "Z",
    }

    return apj


def unpack_firmware(apj_path: str, output_path: str) -> None:
    """Extract binary firmware from APJ file."""
    with open(apj_path, 'r') as f:
        apj = json.load(f)

    compressed = base64.b64decode(apj['image'])
    firmware = zlib.decompress(compressed)

    with open(output_path, 'wb') as f:
        f.write(firmware)

    print(f"Extracted {len(firmware)} bytes to {output_path}")
    print(f"Board ID: {apj['board_id']}")
    print(f"Git: {apj.get('git_identity', 'unknown')}")
    print(f"SHA256: {apj.get('sha256', 'unknown')}")


def main():
    if len(sys.argv) < 3:
        print("Usage:")
        print(f"  {sys.argv[0]} pack <firmware.bin> <board_id> [board_name] > output.apj")
        print(f"  {sys.argv[0]} unpack <firmware.apj> <output.bin>")
        print()
        print("Board IDs (from ArduPilot board_types.txt):")
        print("  140 = MatekH743")
        print("  141 = MatekH743-bdshot")
        print("  186 = CubeOrangePlus")
        print("  1078 = Pixhawk6X")
        print("  1098 = SpeedyBeeF405Wing")
        sys.exit(1)

    cmd = sys.argv[1]

    if cmd == "pack":
        bin_path = sys.argv[2]
        board_id = int(sys.argv[3])
        board_name = sys.argv[4] if len(sys.argv) > 4 else "Meridian"

        apj = pack_firmware(bin_path, board_id, board_name)
        print(json.dumps(apj, indent=2))

    elif cmd == "unpack":
        apj_path = sys.argv[2]
        output_path = sys.argv[3]
        unpack_firmware(apj_path, output_path)

    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)


if __name__ == "__main__":
    main()
