# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A small personal workspace for code-based 3D CAD with **build123d** (on the OpenCascade/OCP kernel), viewed through the **OCP CAD Viewer** VS Code extension (`ocp_vscode`). There is no package, build system, test suite, linter or git repo — just standalone Python scripts. File names and design docs are largely in Finnish.

## Environment and running

- Python 3.14 virtualenv at `.venv/` (VS Code is configured to use it). Key packages: `build123d`, `cadquery-ocp-novtk`, `ocp_vscode`, `ocp-tessellate`, `ezdxf`.
- Run a script: `.venv/bin/python <script>.py`
- Install a package: `.venv/bin/pip install <pkg>`
- `show(...)` from `ocp_vscode` sends geometry to the OCP CAD Viewer panel in VS Code; it needs the viewer extension running. For headless checks, print properties instead (e.g. `part.volume`, `part.bounding_box()`), as in `3d-design-test.py`.
- `ocp_vscode_demo.py` uses `# %%` cells (Jupyter-style, run with shift-enter in VS Code) and shows both build123d styles: **Builder mode** (`with BuildPart() as bp:`) and **Algebra mode** (`b = Box(...); b = fillet(b.edges(), r)`).
- Units are millimetres.

## Kumiveivi (rubber-band winder) reference package

`Kumiveivi_v4_STEP_ja_yhteenveto.zip` holds a finished, non-parametric design: a 1:20 rubber-motor winder gearbox (STEP only, no feature history). `unzip` is not installed; read it with Python's `zipfile`. Contents:

- `Kumiveivi_v4_vaihteisto.step` — assembly: U-housing + 4 gears in position.
- `Kumiveivi_v4_elektroniikan_tilavaraukset.step` — same plus placeholder volumes for a 9 V battery, Arduino Nano, microswitch and display.
- `osat_STEP/01..08_*.step` — the 8 individual parts, each in its own local coordinates.
- `Kumiveivi_v4_yhteenveto.md` — Finnish summary (dimensions, shaft positions, gear table, open issues).
- `lahdetiedot/parameters.json` — source parameters. **The top-level gear params (addendum 1.15m, backlash 0.2) are v3 values; the `v4_modifications` section overrides them for parts 03 and 04.**

Key design facts if modelling around it: gear train 30T→12T / 32T→12T / 36T→12T (= 20:1), module 1.28 mm, 20° pressure angle, face width 6.4 mm, all bores Ø5.0 mm nominal, housing 111.5 × 84.5 × 38.4 mm, gear planes Z A 5.6–12.0 / B 16.0–22.4 / C 26.4–32.8 mm, shaft XY centres listed in `parameters.json`. Not yet designed: shafts, gear-to-shaft locking, axial retention, fasteners, bearings, rubber-motor hook. Enlarging the 12T pinion OD by +8 mm while keeping shaft positions is known to jam — that requires re-laying out the shafts.

Printing guidance from the source: 0.4 mm nozzle, 0.16 mm layers, ≥4 walls, gear face on the bed, 100% infill for small pinions.


# CAD project

Use Python and build123d for all CAD geometry.

Python environment:
.venv

Always:
- use millimeters
- create parametric models
- keep important dimensions as variables at the top of the file
- run the generated Python file after editing it
- fix all Python/build123d errors
- export final models to STEP
- export flat laser-cut parts to DXF
- export 3D-printable parts to STL
- use ocp_vscode show() for visualization

Do not use FreeCAD Python APIs.
Use build123d unless I explicitly request something else.