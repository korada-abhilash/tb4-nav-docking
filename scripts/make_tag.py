#!/usr/bin/env python3
"""Render an AprilTag (tag36h11) to PNG using the installed AprilTag C library (apriltag_to_image),
so the texture on the simulated dock is exactly what apriltag_ros detects.

Usage (in the container): python3 /ws/scripts/make_tag.py [id=0] [out=/ws/models/dock_apriltag/materials/textures/tag36h11_0.png]
The PNG includes the tag's 1-cell white border; its black square is 8/10 of the image width.
"""
import ctypes
import sys
from pathlib import Path

from PIL import Image

TAG_ID = int(sys.argv[1]) if len(sys.argv) > 1 else 0
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else f"/ws/models/dock_apriltag/materials/textures/tag36h11_{TAG_ID}.png")


class ImageU8(ctypes.Structure):
    _fields_ = [("width", ctypes.c_int32), ("height", ctypes.c_int32), ("stride", ctypes.c_int32),
                ("buf", ctypes.POINTER(ctypes.c_uint8))]


lib = ctypes.CDLL("/opt/ros/jazzy/lib/x86_64-linux-gnu/libapriltag.so")
lib.tag36h11_create.restype = ctypes.c_void_p
lib.apriltag_to_image.restype = ctypes.POINTER(ImageU8)
lib.apriltag_to_image.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
fam = lib.tag36h11_create()
im = lib.apriltag_to_image(fam, TAG_ID).contents
pix = bytes(im.buf[r * im.stride + c] for r in range(im.height) for c in range(im.width))
tag = Image.frombytes("L", (im.width, im.height), pix)
OUT.parent.mkdir(parents=True, exist_ok=True)
tag.resize((im.width * 40, im.height * 40), Image.NEAREST).convert("RGB").save(OUT)
print(f"tag36h11 id {TAG_ID}: {im.width}x{im.height} cells -> {OUT}")
