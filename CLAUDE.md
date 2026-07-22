# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MeshCutByLine is a C++17 library that splits triangle meshes into simple polygon regions based on per-face `mark` labels. Built on top of VCGlib (vendored in `vcglib/`), it uses an "extended line cutting" algorithm to handle degenerate mesh topology (non-manifold edges, holes) and produce clean polygon boundaries.

## Build Commands

```bash
# Configure (Windows/VS2022)
mkdir build && cd build
cmake -G "Visual Studio 17 2022" -A x64 ..

# Build
cmake --build . --config Release

# Run tests
./Release/test_mesh_cut.exe
```

CMake 3.15+ required. The project uses MSVC `/utf-8` flag for Chinese characters in comments.

## Architecture

The algorithm runs in three phases, orchestrated by `JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge()`:

**Phase 1 — Edge Classification** (`tool/edge_info.h`): `EdgeInfoManager` builds an edge-to-face map and classifies edges as: `CUT_EDGE_NONE`, `CUT_EDGE_MARK_DIFF` (adjacent faces have different marks), `CUT_EDGE_NON_MANIFOLD` (3+ faces share edge), or `CUT_EDGE_BOUNDARY` (only 1 face).

**Phase 2 — Cut and Mark** (`tool/region_marker.h`, `tool/polyline.h`, `tool/cut_plane.h`): `RegionMarker` does BFS flood-fill to find connected same-mark regions respecting cut edges as barriers. `PolylineManager` connects scattered cut edges into continuous polyline chains. `CutPlaneManager` constructs cutting planes from polyline endpoints.

**Phase 3 — Extract Polygons**: Groups faces by `newMark`, extracts boundary edge loops to form simple closed polygons.

Output: vector of `splitReg` structs containing original mark, new mark, triangle indices, face normal, and boundary vertex indices.

## Key Types

- `CMeshOD` (`tool/cmesh.h`): VCGlib TriMesh with OCF (Optional Components Fast) — face-face adjacency, marks, normals. Scalar type is `double`.
- All tool headers in `tool/` use **inline implementations** (header-only).
- Project namespace: `MeshCutByMark`.

## Code Conventions

- C++17 standard
- MSVC-specific: `/utf-8` flag for source files containing Chinese characters
- No test framework — tests use raw `assert` in `tests/test_mesh_cut.cpp` (21 tests)
- VCGlib dependency is vendored (includes Eigen)

## Known Limitations

- `cutTriangleByPlane()` in `tool/cut_plane.h` is a **stub/TODO** — computes signed distances but does not split triangles
- Only the first boundary loop is stored per region (regions with holes lose data)
- Boundary traversal assumes manifold mesh topology
- Polyline extension uses front-insertion into vector (O(n²) — noted as future optimization target)

## Documentation

- `README.md`: Comprehensive documentation in Chinese (background, algorithm, file structure, data structures, build instructions, tests, usage)
- `docs/superpowers/specs/`: Requirements and design specifications
- `docs/superpowers/plans/`: Implementation plan
