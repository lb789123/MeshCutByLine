# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

MeshCutByLine is a C++17 library that splits triangle meshes into simple polygon regions based on per-face `mark` labels. It uses an "extended line cutting" algorithm to handle degenerate mesh topology (non-manifold edges, holes) and produce clean polygon boundaries.

The actual local cutting is delegated to an **external cutter**: `external/cgalLocalMeshCut` (git submodule, GitHub). It provides `JasMeshAddCutLines::AddCutLines` (CGAL corefine) which cuts an isolated local mesh along a line; this repo only orchestrates (extract local mesh -> cut -> merge back -> pick sub-regions -> extract polygons).

## Build Commands (Windows / Ninja + MSVC)

```powershell
# 1. init submodule
git submodule update --init --recursive

# 2. configure (BOOST_ROOT must be set BEFORE configure)
$env:BOOST_ROOT = 'D:\github\boost_1_91_0'
cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1 && cmake -S . -B out/build/ninja -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM="C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe" -DCGAL_DIR=D:/github/CGAL-6.1.1'

# 3. build + run tests (GMP/MPFR DLLs must be on PATH)
cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1 && cmake --build out/build/ninja --target test_mesh_cut -j 8'
$env:PATH = 'D:\github\CGAL-6.1.1\auxiliary\gmp\bin;' + $env:PATH
.\out\build\ninja\test_mesh_cut.exe
```

CMake 3.18+ required (submodule's CMakeLists). MSVC `/utf-8` for Chinese comments; CGAL template-heavy code needs `/bigobj` (already set on `cglmcut`).

## Architecture

`JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge()` runs three phases:

- **Phase 1 - Edge Classification** (`tool/edge_info.h`): `EdgeInfoManager` builds edge->face map and classifies edges: `CUT_EDGE_NONE`, `CUT_EDGE_MARK_DIFF`, `CUT_EDGE_NON_MANIFOLD` (3+ faces), `CUT_EDGE_BOUNDARY` (1 face).
- **Phase 2 - Cut and Mark** (`tool/region_marker.h`, `tool/polyline.h`, `tool/local_mesh_cut.h`): `RegionMarker` flood-fills same-mark regions; `PolylineManager` connects cut edges into polylines; for dangling endpoints of `NON_MANIFOLD` polylines, `LocalMeshCutManager::cutRegion` extracts a local mesh, calls the external cutter `JasMeshAddCutLines::AddCutLines`, merges back (in-place rewrite of split originals, no `SetD`), marks cut edges by breaking FF adjacency, and rebuilds `curFaces`.
- **Phase 3 - Extract Polygons**: groups faces by `newMark` and extracts boundary loops via `jaslmc::SubRegionBoundary` (outer loop first, holes after).

Output: `std::vector<splitReg>` with `mark`, `newMark`, `inTris`, `normal`, `boundlines` (first loop) and `boundaries` (all loops).

## Key Types / Conventions

- `CMeshOD` (`tool/cmesh.h`): VCGlib TriMesh with OCF components; scalar `double`.
- Tool headers in `tool/` are header-only (`inline` implementations); namespace `MeshCutByMark`.
- External cutter contract: `tool/cut_mesh.h` only includes `JasMeshAddCutLines.h` from the submodule; the implementation is linked from `cglmcut`.
- C++17; tests use raw `assert` in `tests/test_mesh_cut.cpp` (30 tests, run in order from `main()`).
- VCGlib (vendored, includes Eigen) and cgalLocalMeshCut (submodule) both ship their own `vcglib/` copy; they are identical, include guards keep them from colliding.

## Known Limitations

- The external cutter is a conservative black box: degenerate cut lines (along mesh edges / through vertices / midpoint on an edge) may no-op (acceptable per phase-2 design).
- `boundlines` keeps only the first loop for backward compatibility; full loops (outer + holes) are in `splitReg::boundaries`.
- Boundary traversal assumes manifold topology.
- Polyline extension uses front-insertion into a vector (O(n^2) - future optimization).

## Documentation

- `README.md`: comprehensive Chinese docs (background, algorithm, build, usage).
- `docs/superpowers/specs/2026-07-21-mesh-cut-by-mark-design.md`: design (aligned with code).
- `docs/superpowers/plans/2026-07-22-phase2-local-mesh-cut.md` + `specs/2026-07-22-phase2-local-mesh-cut-design.md`: phase-2 local mesh cut design/plan.
