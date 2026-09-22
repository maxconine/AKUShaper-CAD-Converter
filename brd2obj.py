#!/usr/bin/env python3
"""
brd2obj.py - Convert a BoardCAD surfboard design (.brd) into a Wavefront .obj mesh.

Standalone: only the Python standard library is required.

BoardCAD .brd format (as written by BoardCAD's BrdWriter / read by BrdReader)
-----------------------------------------------------------------------------
The file is line oriented text.  Scalar fields look like ``p01 : 182.88`` and
the geometry is stored as cubic Bezier splines made of *knots*::

    p32 : (                                   <- outline  (x along board, y = half width)
    (cp [x0,y0, x1,y1, x2,y2] cont other)     <- one knot: anchor, in-handle, out-handle
    ...
    )
    p33 : ( ... )                             <- bottom rocker  (x along board, z up)
    p34 : ( ... )                             <- deck profile   (x along board, z up)
    p35 : (                                   <- list of cross sections
    (p36 <x position> ...                     <- one cross section, (y half width, z above bottom)
    (cp [...] ...)
    )
    ...
    )

All values are centimetres.  Each knot stores three 2-D points: ``points[0]`` is the
anchor the curve passes through, ``points[1]`` is the handle pointing back toward the
previous knot, and ``points[2]`` is the handle pointing forward to the next knot.  The
cubic Bezier segment between knot ``i`` and knot ``i+1`` therefore has control polygon
``P0 = K[i].anchor, P1 = K[i].out_handle, P2 = K[i+1].in_handle, P3 = K[i+1].anchor``.

Some .brd files (those produced by APS3000 / AKU Shaper exports) are encrypted.  They
start with ``%BRD-1.01`` or ``%BRD-1.02`` followed by a 12-byte header, and the rest is
the same text encrypted with PKCS#5 ``PBEWithMD5AndDES`` using a fixed password and
salt (this is exactly what BoardCAD's own reader does).  A tiny pure-Python DES is
included below so those files can be read without extra dependencies.

Surface reconstruction (mirrors BoardCAD's BezierBoard / BezierBoardCrossSection)
-----------------------------------------------------------------------------------
For a station ``x`` along the board:

1. half_width(x)  = outline y at x      (solve the outline Bezier for x)
2. bottom(x)      = bottom rocker z at x
3. thickness(x)   = deck(x) - bottom(x)
4. The two nearest *real* cross sections (the dummy single-point sections at the very
   tail and nose are skipped, as BoardCAD does) are normalised to unit half-width and
   unit thickness, linearly blended by the relative position of ``x`` between them,
   then scaled back up by half_width(x) and thickness(x) and lifted by bottom(x).
5. The blended half section is sampled at equal arc-length steps, mirrored across the
   stringer, and the resulting rings are stitched into quads.  Tail and nose are capped
   with triangle fans so the mesh is closed.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import re
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


# --------------------------------------------------------------------------------------
# Error type
# --------------------------------------------------------------------------------------
class BrdError(Exception):
    """Raised for any problem reading / interpreting a .brd file."""


# --------------------------------------------------------------------------------------
# Minimal DES (decrypt only) + PBEWithMD5AndDES, needed for encrypted .brd files
# --------------------------------------------------------------------------------------
_IP = [58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4,
       62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8,
       57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27, 19, 11, 3,
       61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7]
_FP = [40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31,
       38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29,
       36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,
       34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25]
_E = [32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9, 8, 9, 10, 11, 12, 13,
      12, 13, 14, 15, 16, 17, 16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25,
      24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1]
_P = [16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10,
      2, 8, 24, 14, 32, 27, 3, 9, 19, 13, 30, 6, 22, 11, 4, 25]
_PC1 = [57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18,
        10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60, 52, 44, 36,
        63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22,
        14, 6, 61, 53, 45, 37, 29, 21, 13, 5, 28, 20, 12, 4]
_PC2 = [14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10,
        23, 19, 12, 4, 26, 8, 16, 7, 27, 20, 13, 2,
        41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48,
        44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32]
_SHIFTS = [1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1]
_SBOX = [
    [14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7,
     0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8,
     4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0,
     15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13],
    [15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10,
     3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5,
     0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15,
     13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9],
    [10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8,
     13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1,
     13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7,
     1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12],
    [7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15,
     13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9,
     10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4,
     3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14],
    [2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9,
     14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6,
     4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14,
     11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3],
    [12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11,
     10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8,
     9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6,
     4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13],
    [4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1,
     13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6,
     1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2,
     6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12],
    [13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7,
     1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2,
     7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8,
     2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11],
]


def _permute(bits: List[int], table: Sequence[int]) -> List[int]:
    return [bits[i - 1] for i in table]


def _des_subkeys(key: bytes) -> List[List[int]]:
    """Expand the 8-byte key into the 16 round keys (48 bits each)."""
    kbits = [(key[i // 8] >> (7 - i % 8)) & 1 for i in range(64)]
    cd = _permute(kbits, _PC1)
    c, d = cd[:28], cd[28:]
    subkeys = []
    for s in _SHIFTS:
        c = c[s:] + c[:s]
        d = d[s:] + d[:s]
        subkeys.append(_permute(c + d, _PC2))
    return subkeys


def _des_block(block: bytes, subkeys: Sequence[List[int]]) -> bytes:
    """Run one 64-bit block through the DES Feistel network with the given key order."""
    bits = [(block[i // 8] >> (7 - i % 8)) & 1 for i in range(64)]
    bits = _permute(bits, _IP)
    left, right = bits[:32], bits[32:]
    for k in subkeys:
        expanded = [a ^ b for a, b in zip(_permute(right, _E), k)]
        out = []
        for i in range(8):
            chunk = expanded[i * 6:(i + 1) * 6]
            row = (chunk[0] << 1) | chunk[5]
            col = (chunk[1] << 3) | (chunk[2] << 2) | (chunk[3] << 1) | chunk[4]
            val = _SBOX[i][row * 16 + col]
            out += [(val >> 3) & 1, (val >> 2) & 1, (val >> 1) & 1, val & 1]
        f = _permute(out, _P)
        left, right = right, [a ^ b for a, b in zip(left, f)]
    bits = _permute(right + left, _FP)
    return bytes(sum(bits[i * 8 + j] << (7 - j) for j in range(8)) for i in range(8))


def des_cbc_decrypt(data: bytes, key: bytes, iv: bytes) -> bytes:
    """DES-CBC decrypt and strip PKCS#5 padding."""
    if len(data) % 8 != 0 or not data:
        raise BrdError("encrypted payload length is not a multiple of the DES block size")
    subkeys = _des_subkeys(key)[::-1]  # decryption = round keys in reverse order
    out = bytearray()
    prev = iv
    for i in range(0, len(data), 8):
        block = data[i:i + 8]
        out += bytes(a ^ b for a, b in zip(_des_block(block, subkeys), prev))
        prev = block
    pad = out[-1]
    if not 1 <= pad <= 8 or any(b != pad for b in out[-pad:]):
        raise BrdError("decryption produced invalid padding (unknown key or corrupt file)")
    return bytes(out[:-pad])


def pbe_md5_des_decrypt(data: bytes, password: str, salt: bytes, iterations: int) -> bytes:
    """PKCS#5 v1.5 PBEWithMD5AndDES (Java's default PBE) decryption."""
    derived = password.encode("latin-1") + salt
    for _ in range(iterations):
        derived = hashlib.md5(derived).digest()
    return des_cbc_decrypt(data, key=derived[:8], iv=derived[8:16])


# These constants are taken from BoardCAD's BrdReader.loadEncryptedFile().
_BRD_SALT = bytes([0xC7, 0x73, 0x21, 0x8C, 0x7E, 0xC8, 0xEE, 0x99])
_BRD_ITERATIONS = 20
_BRD_HEADER_LEN = 12
_BRD_KEYS = {"%BRD-1.02": "deltaXTaildeltaXMiddle", "%BRD-1.01": "deltaXTail"}


def load_brd_text(path: str) -> str:
    """Return the .brd file as plain text, transparently decrypting if needed."""
    if not os.path.isfile(path):
        raise BrdError(f"input file not found: {path}")
    try:
        with open(path, "rb") as fh:
            raw = fh.read()
    except OSError as exc:
        raise BrdError(f"cannot read {path}: {exc}") from exc
    if not raw.strip():
        raise BrdError(f"input file is empty: {path}")

    for magic, password in _BRD_KEYS.items():
        if raw.startswith(magic.encode("ascii")):
            try:
                plain = pbe_md5_des_decrypt(raw[_BRD_HEADER_LEN:], password, _BRD_SALT, _BRD_ITERATIONS)
            except BrdError as exc:
                raise BrdError(f"failed to decrypt {magic} file: {exc}") from exc
            return plain.decode("latin-1")
    return raw.decode("utf-8", errors="replace")


# --------------------------------------------------------------------------------------
# Bezier data model
# --------------------------------------------------------------------------------------
@dataclass
class Knot:
    """A BoardCAD Bezier knot: the anchor plus the incoming and outgoing handles."""
    anchor: Point
    in_handle: Point
    out_handle: Point

    def scaled(self, sx: float, sy: float) -> "Knot":
        return Knot((self.anchor[0] * sx, self.anchor[1] * sy),
                    (self.in_handle[0] * sx, self.in_handle[1] * sy),
                    (self.out_handle[0] * sx, self.out_handle[1] * sy))


def _lerp_pt(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def cubic_bezier(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    """Evaluate a cubic Bezier using the Bernstein form:
    B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3.
    """
    u = 1.0 - t
    b0, b1, b2, b3 = u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t
    return (b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
            b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1])


@dataclass
class Spline:
    """A chain of cubic Bezier segments defined by BoardCAD knots."""
    knots: List[Knot] = field(default_factory=list)

    def segments(self):
        """Yield (P0, P1, P2, P3) control polygons for each consecutive knot pair."""
        for a, b in zip(self.knots, self.knots[1:]):
            yield a.anchor, a.out_handle, b.in_handle, b.anchor

    def polyline(self, samples_per_segment: int = 64) -> List[Point]:
        """Densely sample the whole spline into a polyline (used for arc-length work)."""
        pts: List[Point] = []
        for seg in self.segments():
            start = 0 if not pts else 1  # avoid duplicating shared anchors
            for i in range(start, samples_per_segment + 1):
                pts.append(cubic_bezier(*seg, i / samples_per_segment))
        if not pts and self.knots:
            pts.append(self.knots[0].anchor)
        return pts

    def values_at_x(self, x: float) -> List[float]:
        """All y values where the spline crosses the vertical line at ``x``.

        Each segment of a BoardCAD outline/rocker curve is monotonic in x, so the
        cubic x(t) = x is solved with bisection on t.  Vertical segments (used for
        squash tails / flat noses) contribute both of their end y values.
        """
        ys: List[float] = []
        for p0, p1, p2, p3 in self.segments():
            lo, hi = min(p0[0], p3[0]), max(p0[0], p3[0])
            if not (lo - 1e-9 <= x <= hi + 1e-9):
                continue
            if hi - lo < 1e-9:  # vertical segment
                ys.extend([p0[1], p3[1]])
                continue
            ta, tb = 0.0, 1.0
            increasing = p3[0] > p0[0]
            for _ in range(60):  # bisection: 60 halvings is well beyond double precision
                tm = 0.5 * (ta + tb)
                xm = cubic_bezier(p0, p1, p2, p3, tm)[0]
                if (xm < x) == increasing:
                    ta = tm
                else:
                    tb = tm
            ys.append(cubic_bezier(p0, p1, p2, p3, 0.5 * (ta + tb))[1])
        return ys

    def x_range(self) -> Tuple[float, float]:
        xs = [k.anchor[0] for k in self.knots]
        return min(xs), max(xs)


def hull_value(spline: Spline, x: float, pick) -> float:
    """y at x, using ``pick`` (max or min) when several branches cross the same x.

    x is clamped to the spline's range so tiny floating point overshoot at the
    tail/nose never yields "no segment found".
    """
    lo, hi = spline.x_range()
    ys = spline.values_at_x(min(max(x, lo), hi))
    if not ys:
        raise BrdError(f"curve has no value at x={x:.4f}")
    return pick(ys)


# --------------------------------------------------------------------------------------
# .brd parsing
# --------------------------------------------------------------------------------------
@dataclass
class CrossSection:
    position: float  # x along the board (cm)
    spline: Spline


@dataclass
class Board:
    length: float
    outline: Spline
    bottom: Spline
    deck: Spline
    cross_sections: List[CrossSection]
    name: str = ""


_CP_RE = re.compile(r"\(cp\s*\[([^\]]*)\]")
_FIELD_RE = re.compile(r"^p(\d+)\s*:\s*(.*)$")


def _parse_knot(line: str) -> Knot:
    m = _CP_RE.match(line)
    if not m:
        raise BrdError(f"malformed control point line: {line!r}")
    try:
        v = [float(s) for s in m.group(1).split(",")]
    except ValueError as exc:
        raise BrdError(f"non-numeric control point in line: {line!r}") from exc
    if len(v) != 6:
        raise BrdError(f"control point needs 6 numbers, got {len(v)}: {line!r}")
    # BoardCAD order: points[0]=anchor, points[1]=handle to previous, points[2]=handle to next
    return Knot(anchor=(v[0], v[1]), in_handle=(v[2], v[3]), out_handle=(v[4], v[5]))


def _read_spline_block(lines: List[str], i: int) -> Tuple[Spline, int]:
    """Read ``(cp ...)`` lines (and skip an optional ``gps : ( ... )`` guide-point block)
    until the closing ``)``.  Returns the spline and the index after the block."""
    spline = Spline()
    n = len(lines)
    while i < n:
        s = lines[i].strip()
        if s.startswith("(cp"):
            spline.knots.append(_parse_knot(s))
            i += 1
        elif s.startswith("gps"):
            # guide points are editor helpers only; skip through their closing paren
            i += 1
            while i < n and not lines[i].strip().startswith(")"):
                i += 1
            i += 1
        elif s.startswith(")"):
            return spline, i + 1
        elif not s:
            i += 1
        else:
            raise BrdError(f"unexpected line inside Bezier block: {s!r}")
    raise BrdError("unterminated Bezier block (missing ')')")


def parse_brd(text: str) -> Board:
    """Parse BoardCAD .brd text into a Board with outline/bottom/deck/cross sections."""
    lines = text.splitlines()
    length: Optional[float] = None
    name = ""
    outline = bottom = deck = None
    sections: List[CrossSection] = []

    i = 0
    while i < len(lines):
        s = lines[i].strip()
        m = _FIELD_RE.match(s)
        if not m:
            i += 1
            continue
        fid, val = int(m.group(1)), m.group(2).strip()
        if fid == 1:
            try:
                length = float(val)
            except ValueError:
                pass
            i += 1
        elif fid == 8:
            name = val.strip('"')
            i += 1
        elif fid in (32, 33, 34):
            if not val.startswith("("):
                raise BrdError(f"p{fid} should start a '(' block")
            spline, i = _read_spline_block(lines, i + 1)
            if fid == 32:
                outline = spline
            elif fid == 33:
                bottom = spline
            else:
                deck = spline
        elif fid == 35:
            i += 1
            while i < len(lines):
                s = lines[i].strip()
                if s.startswith("(p36"):
                    parts = s.split()
                    try:
                        pos = float(parts[1])
                    except (IndexError, ValueError) as exc:
                        raise BrdError(f"bad cross section header: {s!r}") from exc
                    spline, i = _read_spline_block(lines, i + 1)
                    sections.append(CrossSection(pos, spline))
                elif s.startswith(")"):
                    i += 1
                    break
                else:
                    i += 1
        else:
            i += 1

    if outline is None or len(outline.knots) < 2:
        raise BrdError("no outline curve (p32) found in file")
    if bottom is None or len(bottom.knots) < 2:
        raise BrdError("no bottom rocker curve (p33) found in file")
    if deck is None or len(deck.knots) < 2:
        raise BrdError("no deck curve (p34) found in file")
    if not sections:
        raise BrdError("no cross sections (p35/p36) found in file")

    sections.sort(key=lambda c: c.position)
    if length is None or length <= 0:
        length = outline.x_range()[1]  # fall back to the outline's extent
    return Board(length, outline, bottom, deck, sections, name)


# --------------------------------------------------------------------------------------
# Surface evaluation
# --------------------------------------------------------------------------------------
MIN_DIM = 0.1  # cm - floor for width/thickness so tips never collapse to zero-area faces


@dataclass
class NormSection:
    """A cross section scaled to unit half-width and unit thickness."""
    position: float
    spline: Spline


def normalise_sections(board: Board) -> List[NormSection]:
    """Drop the dummy single-point tail/nose sections and normalise the rest.

    BoardCAD normalises by ``getWidth()`` (2 * max x of the curve) and
    ``getCenterThickness()`` (deck-centre y minus bottom-centre y).
    """
    result = []
    for cs in board.cross_sections:
        if len(cs.spline.knots) < 2:
            continue
        half_w = max(p[0] for p in cs.spline.polyline(32))
        thick = cs.spline.knots[-1].anchor[1] - cs.spline.knots[0].anchor[1]
        if half_w < 1e-6 or thick < 1e-6:
            continue
        result.append(NormSection(cs.position, Spline([k.scaled(1.0 / half_w, 1.0 / thick) for k in cs.spline.knots])))
    if not result:
        raise BrdError("no usable cross sections (all are degenerate)")
    return result


def resample_by_arclength(spline: Spline, count: int) -> List[Point]:
    """Return ``count`` points spread at equal arc-length steps along the spline.

    Equal *parameter* steps bunch up where handles are long, so we densely sample
    the curve, accumulate segment lengths, and invert that mapping instead.
    """
    poly = spline.polyline(64)
    cum = [0.0]
    for a, b in zip(poly, poly[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = cum[-1]
    out: List[Point] = []
    j = 0
    for k in range(count):
        target = total * k / (count - 1)
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        f = 0.0 if seg_len <= 0 else (target - cum[j]) / seg_len
        out.append(_lerp_pt(poly[j], poly[j + 1], f))
    return out


def blended_half_section(sections: List[NormSection], x: float, count: int) -> List[Point]:
    """Unit-size half section at station x, blended from its two neighbours.

    Like BoardCAD, positions before the first real section or after the last one
    simply reuse that section's shape (t is clamped to 0 / 1).
    """
    if x <= sections[0].position or len(sections) == 1:
        a = b = sections[0]
        t = 0.0
    elif x >= sections[-1].position:
        a = b = sections[-1]
        t = 0.0
    else:
        idx = max(i for i, s in enumerate(sections) if s.position <= x)
        a, b = sections[idx], sections[idx + 1]
        t = (x - a.position) / (b.position - a.position)

    if a is b or t == 0.0:
        return resample_by_arclength(a.spline, count)

    if len(a.spline.knots) == len(b.spline.knots):
        # Same topology: interpolate the control points themselves (BoardCAD's approach),
        # which keeps the rail shape crisp while it morphs between sections.
        knots = [Knot(_lerp_pt(ka.anchor, kb.anchor, t),
                      _lerp_pt(ka.in_handle, kb.in_handle, t),
                      _lerp_pt(ka.out_handle, kb.out_handle, t))
                 for ka, kb in zip(a.spline.knots, b.spline.knots)]
        return resample_by_arclength(Spline(knots), count)

    # Different knot counts: fall back to blending the sampled curves point by point.
    pa = resample_by_arclength(a.spline, count)
    pb = resample_by_arclength(b.spline, count)
    return [_lerp_pt(p, q, t) for p, q in zip(pa, pb)]


def station_ring(board: Board, sections: List[NormSection], x: float, half_points: int) -> List[Tuple[float, float, float]]:
    """Full closed cross-section ring at station x as (x, y, z) points in cm.

    Ring order: bottom centre -> starboard rail -> deck centre -> port rail -> back.
    ``half_points`` is the number of samples along one half (bottom centre to deck
    centre inclusive); the ring therefore has ``2 * half_points - 2`` vertices.
    """
    half_w = max(hull_value(board.outline, x, max), MIN_DIM / 2)
    z_bottom = hull_value(board.bottom, x, min)
    z_deck = hull_value(board.deck, x, max)
    thickness = max(z_deck - z_bottom, MIN_DIM)

    half = blended_half_section(sections, x, half_points)
    # Scale unit section back to real size and lift it onto the rocker.
    right = [(x, py * half_w, pz * thickness + z_bottom) for py, pz in half]
    # Mirror across the stringer (y -> -y), skipping the two centre points already present.
    left = [(x, -py, pz) for (_, py, pz) in reversed(right[1:-1])]
    return right + left


# --------------------------------------------------------------------------------------
# Mesh construction and OBJ export
# --------------------------------------------------------------------------------------
Vec3 = Tuple[float, float, float]


def build_mesh(board: Board, stations: int, half_points: int) -> Tuple[List[Vec3], List[List[int]]]:
    """Loft the station rings into a closed quad/tri mesh.  Returns (vertices, faces)
    with 0-based vertex indices."""
    if stations < 2 or half_points < 3:
        raise BrdError("need at least 2 stations and 3 points per half section")

    sections = normalise_sections(board)
    length = board.length
    xs = [length * i / (stations - 1) for i in range(stations)]

    verts: List[Vec3] = []
    rings: List[List[int]] = []
    for x in xs:
        ring = station_ring(board, sections, x, half_points)
        rings.append(list(range(len(verts), len(verts) + len(ring))))
        verts.extend(ring)

    faces: List[List[int]] = []
    m = len(rings[0])
    # Side walls: one quad per ring edge between consecutive stations.
    for r0, r1 in zip(rings, rings[1:]):
        for j in range(m):
            k = (j + 1) % m
            faces.append([r0[j], r0[k], r1[k], r1[j]])

    # Caps: triangle fan from the ring centroid so the mesh is watertight.
    for ring, flip in ((rings[0], True), (rings[-1], False)):
        pts = [verts[i] for i in ring]
        centre = tuple(sum(p[c] for p in pts) / len(pts) for c in range(3))
        ci = len(verts)
        verts.append(centre)  # type: ignore[arg-type]
        for j in range(m):
            k = (j + 1) % m
            tri = [ci, ring[k], ring[j]] if flip else [ci, ring[j], ring[k]]
            faces.append(tri)

    _orient_outward(verts, faces, rings)
    return verts, faces


def _newell_normal(pts: Sequence[Vec3]) -> Vec3:
    nx = ny = nz = 0.0
    for a, b in zip(pts, pts[1:] + [pts[0]]):
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return (nx, ny, nz)


def _orient_outward(verts: List[Vec3], faces: List[List[int]], rings: List[List[int]]) -> None:
    """Make every face wind counter-clockwise when seen from outside the board.

    All side quads share one winding convention, so we test a single quad on the
    deck near mid-length; each cap is tested against its own axial direction.
    """
    n_side = (len(rings) - 1) * len(rings[0])
    mid = len(rings) // 2
    deck_j = len(rings[0]) // 2 - 1  # index just before the deck-centre vertex
    probe = faces[mid * len(rings[0]) + deck_j]
    if _newell_normal([verts[i] for i in probe])[2] < 0:  # deck normal should point +z
        for f in faces[:n_side]:
            f.reverse()
    m = len(rings[0])
    tail_faces = faces[n_side:n_side + m]
    nose_faces = faces[n_side + m:]
    if _newell_normal([verts[i] for i in tail_faces[0]])[0] > 0:  # tail cap should face -x
        for f in tail_faces:
            f.reverse()
    if _newell_normal([verts[i] for i in nose_faces[0]])[0] < 0:  # nose cap should face +x
        for f in nose_faces:
            f.reverse()


UNIT_SCALE = {"mm": 10.0, "cm": 1.0, "m": 0.01, "in": 1.0 / 2.54}


def write_obj(path: str, verts: Sequence[Vec3], faces: Sequence[Sequence[int]], board: Board,
              units: str, z_up: bool) -> None:
    scale = UNIT_SCALE[units]
    out_dir = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(out_dir):
        raise BrdError(f"output directory does not exist: {out_dir}")
    try:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("# Generated by brd2obj.py from BoardCAD .brd\n")
            fh.write(f"# board: {board.name or 'unnamed'}  length: {board.length:.2f} cm  units: {units}\n")
            fh.write(f"# axes: X = tail->nose, {'Z' if z_up else 'Y'} = up\n")
            fh.write(f"o {re.sub(r'[^A-Za-z0-9_.-]', '_', board.name) or 'surfboard'}\n")
            for x, y, z in verts:
                # BoardCAD frame is X forward, Y across, Z up.  OBJ viewers usually expect
                # Y up, so by default rotate -90 deg about X:  (x, y, z) -> (x, z, -y).
                if z_up:
                    fh.write(f"v {x * scale:.6f} {y * scale:.6f} {z * scale:.6f}\n")
                else:
                    fh.write(f"v {x * scale:.6f} {z * scale:.6f} {-y * scale:.6f}\n")
            fh.write("s 1\n")
            for f in faces:
                fh.write("f " + " ".join(str(i + 1) for i in f) + "\n")  # OBJ indices are 1-based
    except OSError as exc:
        raise BrdError(f"cannot write {path}: {exc}") from exc


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a BoardCAD .brd surfboard design to a Wavefront .obj mesh.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("input", help="path to the .brd file (plain or encrypted BoardCAD format)")
    ap.add_argument("output", help="path of the .obj file to write")
    ap.add_argument("--stations", type=int, default=200,
                    help="number of cross-section rings along the length")
    ap.add_argument("--ring-points", type=int, default=64,
                    help="points per half section (bottom centre to deck centre)")
    ap.add_argument("--units", choices=sorted(UNIT_SCALE), default="mm", help="output length unit")
    ap.add_argument("--z-up", action="store_true",
                    help="keep BoardCAD's Z-up frame instead of converting to Y-up")
    ap.add_argument("--dump-brd", metavar="TXT",
                    help="also write the decoded plain-text .brd contents to this path")
    args = ap.parse_args(argv)

    try:
        text = load_brd_text(args.input)
        if args.dump_brd:
            with open(args.dump_brd, "w", encoding="utf-8") as fh:
                fh.write(text)
        board = parse_brd(text)
        verts, faces = build_mesh(board, args.stations, args.ring_points)
        write_obj(args.output, verts, faces, board, args.units, args.z_up)
    except BrdError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    real = [c for c in board.cross_sections if len(c.spline.knots) >= 2]
    print(f"Read {args.input}: length {board.length:.2f} cm, "
          f"{len(board.outline.knots)} outline knots, {len(real)} cross sections at "
          + ", ".join(f"{c.position:.1f}" for c in real) + " cm")
    print(f"Wrote {args.output}: {len(verts)} vertices, {len(faces)} faces ({args.units}, "
          f"{'Z' if args.z_up else 'Y'}-up)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
