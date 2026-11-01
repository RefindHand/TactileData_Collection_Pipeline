#!/usr/bin/env python3
"""Convert the official Gemini 305 CAD (STEP) to an STL mesh for MuJoCo. Run once.

Needs OpenCascade bindings (only for this script): pip install cadquery-ocp
  python3 convert_cad.py "<...>/Gemini 305 CAD_V1.1/3D/3D__00_mb2502_asm(G305)_ V1.1.stp"

The STEP assembly has 4 top-level shapes; only the housing (shape 0, includes the
front glass) is exported. Shapes 1-2 are the lenses hidden behind the glass and
shape 3 is a surface-only compound (FOV cones, connector), which would block the
virtual cameras. Output keeps the CAD frame and units (mm); gripper_camera/model/gemini305.xml
scales and places it.
"""
import argparse
from pathlib import Path

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopoDS import TopoDS_Iterator

MESH_OUTPUT = Path(__file__).parent / 'gripper_camera' / 'model' / 'meshes' / 'gemini305_housing.stl'
HOUSING_INDEX = 0
LINEAR_DEFLECTION_MM = 0.1
ANGULAR_DEFLECTION_RAD = 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('step', help='Gemini 305 CAD_V1.1 .stp file')
    parser.add_argument('-o', '--output', default=str(MESH_OUTPUT))
    args = parser.parse_args()

    reader = STEPControl_Reader()
    if reader.ReadFile(args.step) != 1:
        raise SystemExit(f'failed to read {args.step}')
    reader.TransferRoots()
    shapes = []
    it = TopoDS_Iterator(reader.OneShape())
    while it.More():
        shapes.append(it.Value())
        it.Next()

    housing = shapes[HOUSING_INDEX]
    BRepMesh_IncrementalMesh(housing, LINEAR_DEFLECTION_MM, False, ANGULAR_DEFLECTION_RAD, True)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(housing, args.output):
        raise SystemExit(f'failed to write {args.output}')
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
