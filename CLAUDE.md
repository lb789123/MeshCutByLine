# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

MeshCutByLine is a C++17 library that splits triangle meshes into simple polygon regions based on per-face `mark` labels. It uses an "extended line cutting" algorithm to handle degenerate mesh topology (non-manifold edges, holes) and produce clean polygon boundaries.

The actual local cutting is delegated to an **external cutter**: `external/cgalLocalMeshCut` (git submodule, GitHub). It provides `JasMeshAddCutLines::AddCutLines` (CGAL corefine) which cuts an isolated local mesh along a line; this repo only orchestrates (extract local mesh -> cut -> merge back -> pick sub-regions -> extract polygons).

Since 2026-08 the cutter contract was hardened: corefine output faces carry a `f:source` face property (propagated through a `visitor`) so pieces are assigned back to their parent faces directly, zero-area slivers are **folded** (collapsed) instead of skipped, and the pipeline guarantees the local mesh stays **manifold** after every cut (no duplicate vertices / non-manifold edges or vertices), because CGAL corefine is not reliable on non-manifold input.

## Build Commands (Windows / Ninja + MSVC)

```powershell
# 1. init submodule
git submodule update --init --recursive

# 2. configure (BOOST_ROOT must be set BEFORE configure; Visual Studio generator,
#    Ninja also works: -G Ninja -DCMAKE_BUILD_TYPE=Debug)
$env:BOOST_ROOT = 'D:\github\boost_1_91_0'
cmake -S . -B build -G "Visual Studio 17 2022" -DCGAL_DIR=D:/github/CGAL-6.1.1

# 3. build + run tests (GMP/MPFR DLLs must be on PATH)
cmake --build build --config Release --target test_mesh_cut
$env:PATH = 'D:\github\CGAL-6.1.1\auxiliary\gmp\bin;' + $env:PATH
.\build\Release\test_mesh_cut.exe
```

CMake 3.18+ required (submodule's CMakeLists). MSVC `/utf-8` for Chinese comments; CGAL template-heavy code needs `/bigobj` (already set on `cglmcut`).

## Architecture

`JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge()` runs three phases:

- **Phase 1 - Edge Classification** (`tool/edge_info.h`): `EdgeInfoManager` builds edge->face map and classifies edges: `CUT_EDGE_NONE`, `CUT_EDGE_MARK_DIFF`, `CUT_EDGE_NON_MANIFOLD` (3+ faces), `CUT_EDGE_BOUNDARY` (1 face).
- **Phase 2 - Local Cut and Stitch** (`tool/region_marker.h`, `tool/polyline.h`, `tool/local_mesh_cut.h`, `tool/polygon_mesh.h`, submodule `cgalLocalMeshCut`): `RegionMarker` flood-fills same-mark regions; `PolylineManager` connects cut edges into polylines. Two cut paths, selected by the external `SetCutPathMode` parameter (`CUT_PATH_POLYGON`, the default / `CUT_PATH_TRIANGLE`); both deliver the same `retRegs` output, they differ in whether original triangles get cut. Per-region safety fallback: in polygon mode, regions with a global star vertex or a multi-loop (hole) boundary fall back to the triangle path:
  - **Polygon path** (`CUT_PATH_POLYGON`): `prepareLocalCutPolygon` (parallel) builds one n-gon `ExactMesh` from the region's boundary loop (`jaslmc::CreateExactMesh`) and cuts it per line (`jaslmc::CutMeshExact`, result in `exact.polyResult` — no `f:global`/`v:g`, no seams). `mergeLocalCutPolygon` (serial) merges independently: maps poly vertices to global by exact coordinates (cut points appended as orphan vertices, faces untouched), distributes the region's original faces to same-mark connected groups by exact centroid point-in-polygon, assigns newMarks only to non-empty groups (never orphans), records boundary loops in `LocalCutResult::polyLoops` and appended cut points in `orphanCutPoints`. No stitching; instead `conformSharedEdgeCutPoints` (anonymous namespace in JasMeshMarkAndCutSplit.cpp, runs after the Phase 3 output loop) canonicalizes near-duplicate cut points on shared edges and splices them into **all** output boundary loops — both polygon-path and legacy — so adjacent regions' loops share the same vertex subdivision (no T-junctions).
  - **Triangle path** (`CUT_PATH_TRIANGLE`, also the fallback): `prepareLocalCut` (parallel, `jaslmc::CutFacesExact` — fills `f:global`/`v:g`, collects seams), `mergeLocalCut` (serial, rewrites/appends faces), `stitchAllSeams` (serial, merges seam cut points, pure-splits external neighbor faces).
  Both paths share the exact-coordinate → global vertex index for cut-point dedup.
- **Phase 3 - Extract Polygons**: groups faces by `newMark` and extracts boundary loops via `jaslmc::SubRegionBoundary` (outer loop first, holes after).

Output: `std::vector<splitReg>` with `mark`, `newMark`, `inTris`, `normal`, `boundlines` (first loop) and `boundaries` (all loops).

## Key Types / Conventions

- `CMeshOD` (`tool/cmesh.h`): VCGlib TriMesh with OCF components; scalar `double`.
- Tool headers in `tool/` are header-only (`inline` implementations); namespace `MeshCutByMark`.
- External cutter contract: `tool/cut_mesh.h` only includes `JasMeshAddCutLines.h` from the submodule; the implementation is linked from `cglmcut`.
- C++17; tests use raw `assert` + `REQUIRE` in `tests/test_mesh_cut.cpp` (37 tests, run in order from `main()`). The test file `#undef`s `NDEBUG` before `<cassert>` — Release CMake otherwise voids all asserts and the suite validates nothing (this hid the 2026-08 polygon-path regression).
- VCGlib (vendored, includes Eigen) and cgalLocalMeshCut (submodule) both ship their own `vcglib/` copy; they are identical, include guards keep them from colliding.

## Known Limitations

- The external cutter is a conservative black box: degenerate cut lines (along mesh edges / through vertices / midpoint on an edge) may no-op (acceptable per phase-2 design).
- The local mesh passed to corefine **must be manifold**: `CGAL::Surface_mesh` refuses non-manifold edges (`add_face` fails and the face is dropped; `Cut3D` counts them in `CutResult::dropped_input_face_count`), and non-manifold vertices (star vertices) crash corefine (assert in Debug, access violation in Release). The pipeline therefore folds zero-area slivers, merges new vertices with existing ones by exact coordinates, and removes orphan vertices after each cut.
- Seam propagation is a pure split: a seam edge with several new vertices is split once into n+1 sub-faces, and the seam->neighbor map is updated so later vertices keep being propagated; endpoint-coincident new vertices are skipped (the external face already shares that endpoint).
- `boundlines` keeps only the first loop for backward compatibility; full loops (outer + holes) are in `splitReg::boundaries`.
- Polygon path is an approximate face partition: original triangles are never cut; straddling triangles join whichever piece contains their centroid (exact point-in-polygon); sliver pieces with no whole-triangle centroid are dropped (no newMark, not emitted). Cut points are appended to the global mesh as face-less orphan vertices so `boundaries` can reference them; `conformSharedEdgeCutPoints` then splices them consistently into adjacent regions' loops (near-duplicates canonicalized at distance² < 1e-12).
- `jaslmc::CutMeshExact` uses only the first two line-polygon intersections (`cutPts[0]/[1]`): non-convex region polygons crossed 4+ times by a cut line are split incorrectly (submodule follow-up), and dropped sub-polygons (size < 3 / rejected `add_face`) leave geometric gaps (masked by the centroid fallback).
- Boundary traversal assumes manifold topology.
- Polyline extension uses front-insertion into a vector (O(n^2) - future optimization).

## Documentation

- `README.md`: comprehensive Chinese docs (background, algorithm, build, usage, debug output, limitations).
- `docs/superpowers/specs/2026-07-21-mesh-cut-by-mark-design.md`, `docs/superpowers/plans/2026-07-22-phase2-local-mesh-cut.md` and `specs/2026-07-22-phase2-local-mesh-cut-design.md`: historical design/plan records; the code has since evolved (see git log), keep them as historical documentation.
