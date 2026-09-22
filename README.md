# BoardCAD .brd → .obj converter

`brd2obj.py` turns a [BoardCAD](https://havardnj.github.io/boardcad-le/) surfboard design (`.brd`) into a
closed, watertight Wavefront `.obj` mesh that you can open in Blender, MeshLab, Fusion 360, a slicer, etc.

It is a single Python file with **no dependencies** beyond the standard library, and it reads both
plain-text `.brd` files and the encrypted `%BRD-1.01` / `%BRD-1.02` variant produced by AKU Shaper /
APS3000 exports.

## Requirements

- Python 3.8 or newer (no packages to install)

## Usage

```bash
python3 brd2obj.py input.brd output.obj
```

Example with this repo's sample board:

```bash
python3 brd2obj.py gskateMax.brd gskateMax.obj
# Read gskateMax.brd: length 182.88 cm, 5 outline knots, 4 cross sections at 2.5, 30.5, 91.4, 152.4 cm
# Wrote gskateMax.obj: 25202 vertices, 25326 faces (mm, Y-up)
```

### Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--stations N` | `200` | Number of cross-section rings along the length. More = smoother rocker/outline. |
| `--ring-points N` | `64` | Points per half section (bottom centre → rail → deck centre). More = smoother rails. |
| `--units mm\|cm\|m\|in` | `mm` | Output length unit. BoardCAD stores centimetres internally. |
| `--z-up` | off | Keep BoardCAD's frame (X tail→nose, Y across, Z up). Default converts to Y-up, which most OBJ viewers expect. |
| `--dump-brd FILE` | – | Also write the decoded plain-text `.brd` to `FILE` (handy for inspecting encrypted files). |

Run `python3 brd2obj.py -h` for the full help text.

### Output

- Vertices (`v`) and faces (`f`) only; quads along the hull, triangle fans capping the tail and nose.
- The mesh is closed and consistently wound (outward normals), so it is suitable for volume
  calculation, 3D printing and boolean operations.
- X runs from the tail (0) to the nose (board length). The board is symmetric about the stringer.

## How it works

1. **Read** the file. If it starts with `%BRD-1.0x` it is decrypted exactly the way BoardCAD's own
   `BrdReader` does (PKCS#5 `PBEWithMD5AndDES`, fixed password/salt, 12-byte header). A small
   pure-Python DES implementation is included so no crypto library is needed.
2. **Parse** the geometry. BoardCAD stores everything as cubic Bezier splines made of knots
   (`(cp [x0,y0, x1,y1, x2,y2] ...)` = anchor, incoming handle, outgoing handle):
   - `p32` outline – (x along board, half width)
   - `p33` bottom rocker and `p34` deck profile – (x along board, height)
   - `p35`/`p36` cross sections – (half width, height above bottom) at given x positions
3. **Evaluate** the surface at each station x, following BoardCAD's `BezierBoard` logic:
   half-width, bottom and deck heights are solved from their curves; the two neighbouring
   cross sections are normalised to unit size, blended by relative position, then scaled to the
   local width and thickness and lifted onto the rocker. Points are spaced by arc length so the
   rail is sampled evenly.
4. **Mesh** the rings: mirror across the stringer, stitch consecutive rings into quads, cap both
   ends, and check winding so all normals face outward.
5. **Write** the `.obj`.

## Errors

The script exits with status 1 and a one-line message for: a missing or empty input file, an
output directory that does not exist, a corrupt/unknown-key encrypted file, or a file with no
outline / rocker / deck / cross-section data.

## Repository contents

| Path | What it is |
| --- | --- |
| `brd2obj.py` | The converter. |
| `gskateMax.brd` | Sample board (6'0" × 20.6" × 2.7", encrypted BoardCAD format). |
| `gskateMax.obj` | Mesh produced from the sample with default settings. |
| `dxf's/` | Outline, profile and slice DXFs exported from BoardCAD for the same board; used to verify the converter (outline/rocker match to 0.00 mm, slices within 0.25 mm). |

## Limitations

- Fins, fin boxes, stringer, and concave/vee that is not captured in the cross sections are not
  modelled — only what BoardCAD's Bezier surface describes.
- Cross sections with differing numbers of knots are blended point-by-point rather than by
  control point, which is slightly less faithful to BoardCAD than the same-knot-count case.
- Only BoardCAD `.brd` files are supported. Shape3d `.s3d` / `.s3dx` files are a different format
  (open them in BoardCAD and save as `.brd` first).

## Licence

Do whatever you like with it. The encryption constants are taken from the open-source BoardCAD
project (GPL), whose `BrdReader.java` documents the file format.
