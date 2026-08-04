# qTracer — CloudCompare Plugin

DFN fracture trace extraction pipeline ported into a CloudCompare Standard plugin.

## What this plugin does

Exposes four `QAction`s:

All user-facing names follow the terminology of the accompanying paper (§2) — see "Paper terminology" below before renaming anything.

- **Color Filtering…** (optional pre-processing) — opens `ColorFilterDlg` with live 3D preview driven by the cloud's visibility array. Single filter: **C = a·R + b·G + c·B** with a/b/c each ∈ [0, 1] (slider + spinbox). The C min/max threshold is picked on a `HistogramRangeWidget` (QCustomPlot-based histogram of C values with two draggable vertical cursors: green=min, red=max), paired with min/max spinboxes for exact input. To filter by a single channel, zero the other two coefficients (e.g. a=1, b=c=0 → R-only). On OK, `partialClone` creates a new sibling cloud `<name> [color filtered C(a,b,c) min-max]`. Non-destructive: Cancel restores the original visibility.
- **Outcrop Area Computation…** — opens `OutcropAreaDlg`. Partitions the cloud and sums the area of each cell's local-plane / cell-box intersection polygon. Two partitioners, picked via a combobox that swaps the parameter pane (`QStackedWidget`):
  - **Octree** (fixed cell size): the dialog asks for a `Cell size` (snapped to the nearest octree level), `Min points per cell`, and a `Min planarity` ((λ2-λ3)/λ1) filter. PCA per cell.
  - **Kd-tree** (adaptive, planarity-driven): uses `ccKdTree::build` with a `Max error (RMS)` split threshold and a `Min points per leaf`. The plane equation is already stored on each leaf (no extra PCA), and planarity is enforced by the tree's split criterion. The "cell box" is the leaf points' AABB.

  Results (total area, valid-cell count, rejection breakdown, actual cell size / max leaf RMS) shown inside the same dialog — Compute can be re-run with different parameters without reopening the dialog. Optionally emits a `ccMesh` "Area patches" sibling of the source cloud containing every counted polygon as fan-triangulated triangles. Primary use: denominator for P21 (= total trace length / outcrop area).
- **P21 Computation…** — opens `P21Dlg`. Scans the DB tree for candidate trace groups (any node whose top-level children carry a 2-vertex polyline descendant — **polyline-based detection**, matches both stage-3 `[trace pieces]` and stage-4 `Traces` groups; contour polylines are excluded by the 2-vertex size check in `findTracePolyline`) and area meshes (any `ccMesh`), pre-selects the ones matching the current DB selection, and computes P21 = `totalPolylineLength / MeshSamplingTools::computeMeshArea(mesh)`. A "manual area value" checkbox lets the user override the mesh pick with a number typed in. Action is always enabled; auto-detection is in the dialog. User picks the convention (per-cluster vs combined) from the combobox.
- **Fracture Extraction (Pipeline)** — the main action. Select one point cloud, open `PipelineDlg`, run all 6 stages sequentially:

1. **Eigenvector Computing** — adds `PC Linearity` + `MaxEigVec_X/Y/Z` scalar fields to the cloud.
2. **Cylindrical DBSCAN Clustering** — custom directional variant (cylinder or sphere neighbourhood, eigenvector alignment check); adds `DBSCAN` SF.
3. **Lineation** — `AutoSegmentationTools::extractConnectedComponents` on the DBSCAN SF → `createTraces` produces `ccFacet`s + trace polylines in a `[trace pieces]` group, one `Trace piece N` per cluster.
4. **Trace Clustering** — region-growing merge of close-collinear trace pieces → `Traces` group of `Merged trace N` nodes.
5. **Plane Fitting** — pairs near-intersecting merged traces and fits a `ccFacet` on the 4 merged-polyline endpoints (P1, Q1, P2, Q2). Output → `Joint Planes` group.
6. **Coplanar Plane Merging** — sequential-RANSAC consolidation of near-coplanar joint planes from stage 5 into fewer, larger plane facets → `Merged Joint Planes` group. Iterates up to `Max passes` (dialog parameter, default 10) — each pass re-runs the merge on the previous pass's output, stopping early when the plane count no longer decreases. Non-destructive: the stage-5 output is kept alongside.

All stage outputs are added under one parent DB node `qTracer [<cloud name>]`.

### Running only part of the pipeline

Each stage in `PipelineDlg` is a checkable `QGroupBox`. The user can tick any **contiguous** range (`PipelineDlg::accept` rejects gaps — e.g. "4 + 6 without 5" is not allowed). Starting at stage ≥ 2 requires the selected cloud to already carry the upstream scalar fields (`PC Linearity` / `MaxEigVec_X/Y/Z` for stage 2; `DBSCAN` additionally for stage 3). Starting at stage ≥ 4 expects the selection to be the corresponding ccHObject group (`[trace pieces]` for 4, `Traces` for 5, `Joint Planes` for 6); the group is consumed as read-only input and is not re-parented under the new result root.

Both the per-stage checkbox states **and every parameter value** persist via `QSettings` (org `CCCorp`, app `CloudCompare`, inherited from CC):
- Stage checks → `qTracer/PipelineDlg/stages/stage{1..6}`
- Parameters   → `qTracer/PipelineDlg/params/{kernelRadius, dbscanRadius, dbscanMinPoints, dbscanMaxAngle, dbscanLinearity, dbscanSearchType, randomColors, coneRadius, twoTraceDist, clusterMinAngle, planeIntersectionDist, planeMinTraceLen, planeMinAngle, planeMaxEndPointDist, mergeMaxNormalAngle, mergeMaxPlaneDist, mergeMaxPasses}`

Writes happen on dialog-accept, not on cancel — so Cancel preserves the previous saved values. On load, missing keys fall back to the widget's `.ui` default (so adding a new parameter later is safe).

## Provenance

Ported from the standalone console app at `E:\Research\DFNExtracter\Tracer`. The original project is kept as a frozen research reference — **don't modify it**. Backup at GitHub `CC-qtracer` (private).

## Non-obvious decisions made during the port

These changes are intentional — don't silently undo them:

- `DBSCANParams` struct — the original `expandCluster` hard-coded `radius=0.03`, `minPoints=50`, `maxAngleDeg=15`, `linearityThreshold=0.9`, `searchType=1` at the top of the function, **overriding the `radius` argument**. Now all 5 flow through `PipelineDlg` via the struct. The override lines were removed so the dialog actually controls behavior.
- `ComputeEigen` no longer calls `runDBSCAN` at its end — each stage is invoked explicitly from `qTracer.cpp::doPipeline()`.
- `<ppl.h>` removed (was an unused include).
- At the start of stage 3 the `DBSCAN` SF is deleted from the source cloud so `partialClone` doesn't copy it into every sub-cloud. Same behavior as the standalone source.
- `GetCloudLinearity` was dead code in the original port; removed.

## Post-port improvements (deviations from the frozen reference)

These are intentional fixes on top of the straight port. `E:\Research\DFNExtracter\Tracer\` does **not** have them.

- **Trace polyline endpoints**: `createTraces` and `TraceClustering` originally computed the polyline as `bbox_center ± MaxEigVec * (0.5 * ‖bbox diagonal‖)`. The bbox diagonal is a property of the axis-aligned bounding box, not of the cluster's extent along its principal axis — so traces were **systematically too long** unless the principal axis happened to align with a world axis. Now both places use `ComputeTraceEndpoints`, which projects every cluster point onto the unit eigenvector about the centroid and returns the extreme projections. The polyline length now equals the real extent of the cluster along its principal axis. Downstream merge & plane-fitting distances therefore behave qualitatively differently from the original.
- **Stage 5 fit cloud**: a "pool all member endpoints" variant (using `PC_die` instead of just the 4 combined-polyline endpoints) plus a planarity rejection filter were tried on 2026-04-23. Neither helped in practice; Stage 5 was reverted to the original 4-endpoint `ccFacet::Create` fit.
- **`expandCluster` SF access**: the original hot loops used `pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"))` repeatedly inside the neighbour loops, and called `GetPointMaxEigVecFromSF` (which itself did 4× the same lookup per call). All SF pointers are now cached once at the top of `expandCluster`; inner loops call `sf->getValue(i)` / `sf->setValue(i, v)` directly. Same semantics, dramatically faster — Stage 2 was dominated by these string lookups on large clouds.
- **Console spam**: per-point `std::cout` prints in `runDBSCAN`, `expandCluster`, `TraceClustering`, and `PlaneFitting` (leftovers from the standalone console app) removed. They were also slowing Stage 2 hugely under Windows console I/O.
- **Stage 4 non-destructive on input**: original `TraceClustering` re-parented each facet's `TipVertices` cloud out of the `[trace pieces]` group and into the working set — a one-shot operation that crashed if Stage 4 was re-run on the same `[trace pieces]` group (which is exactly what happens when starting from stage 4 manually after a previous full run). Fixed by cloning the 2-point `TipVertices` into the working set instead of moving the original. The source `[trace pieces]` group is now read-only.

## File map

- `include/qTracer.h` / `src/qTracer.cpp` — plugin class, two actions (`Color Filtering…` + `Fracture Extraction (Pipeline)`), selection gating, `doColorFilter()` + `doPipeline()`.
- `include/ColorFilterDlg.h` / `src/ColorFilterDlg.cpp` / `ui/ColorFilterDlg.ui` — interactive color-filter dialog. Uses the cloud's visibility array + `redrawDisplay()` for live preview; 75 ms debounce via `QTimer`; reject() restores the pre-dialog visibility state. Histogram is debounced separately (120 ms) and rebuilt only when a/b/c change.
- `include/HistogramRangeWidget.h` / `src/HistogramRangeWidget.cpp` — `QCustomPlot` subclass: histogram bars + two draggable vertical cursors (green/red) for a min/max range picker. Emits `lowerChanged(double)` / `upperChanged(double)`. Registered in `ColorFilterDlg.ui` as a promoted widget (`<customwidgets>`). Linked via `target_link_libraries(qTracer QCustomPlot Qt6::PrintSupport)` in the plugin root CMakeLists.
- `include/OutcropArea.h` / `src/OutcropArea.cpp` — outcrop surface-area estimator with two partitioners (`Params::partitioner` = `Octree` or `KdTree`). Shared core: `planeCubeIntersection` walks the 12 cube/AABB edges collecting sign-change intersections, CCW-sorts them on the plane, fan-triangulates for area. Octree path uses `DgmOctree::executeFunctionForAllCellsAtLevel` with per-cell PCA + planarity filter. Kd-tree path uses `ccKdTree::build(maxError, RMS, minPts, …)` + `getLeaves` — plane equation is pre-computed on each `TrueKdTree::Leaf`, leaf's AABB is derived from its point set. `Result` struct carries total area + rejection breakdown + an optional `ccMesh` of every counted polygon.
- `include/OutcropAreaDlg.h` / `src/OutcropAreaDlg.cpp` / `ui/OutcropAreaDlg.ui` — combined input + result dialog for the outcrop-area action. Result labels live in the dialog; Compute/Close buttons; dialog stays open so the user can retry with different parameters.
- `include/P21Dlg.h` / `src/P21Dlg.cpp` / `ui/P21Dlg.ui` — P21 computation dialog. Walks the DB tree once at construction to populate two `QComboBox`es with all candidate trace groups and area meshes. Trace length is summed via `findTracePolyline` (DFS for a 2-vertex `ccPolyline`) + per-polyline Euclidean segment sum. Area comes from `MeshSamplingTools::computeMeshArea` on the chosen mesh, or a manual value.
- `include/PipelineDlg.h` / `src/PipelineDlg.cpp` / `ui/PipelineDlg.ui` — combined parameter dialog (6 checkable `QGroupBox` sections, one per stage).
- `include/IdentifyFracture.h` / `src/IdentifyFracture.cpp` — algorithm API (`ComputeEigen`, `runDBSCAN`, `expandCluster`, `createTraces`, `TraceClustering`, `PlaneFitting`, `MergeCoplanarPlanes`, helpers). All 6 pipeline entry points accept an optional `GenericProgressCallback*` and drive it via `NormalizedProgress`. Note: stage 3's `extractConnectedComponents` is a CCCoreLib built-in that does not itself take a progress callback — progress during stage 3 only tracks the facet-build loop.
- `include/facetsClassifier.h` — copied from qFacets; defines `c_darkColorRatio` + `FacetsClassifier::GenerateSubfamilyColor`.
- `include/disclaimerDialog.h` — inline `DisclaimerDialog` + `ShowDisclaimer()`. Header-only with a `static` flag; only `qTracer.cpp` includes it (don't include elsewhere — would cause multiple-definition linker errors).

## Build environment

- CloudCompare tree at `C:\CloudCompare\` (upstream, `master` branch). The CC local `.git/info/exclude` has `plugins/core/Standard/qTracer/` so CC's repo ignores this plugin dir.
- `C:\CloudCompare\` is admin-owned → `git config --global --add safe.directory` was set for both the CC root and this subdir.
- CMake option: `PLUGIN_STANDARD_QTRACER=ON`. Target: `qTracer`.
- Qt 6, MSVC (not Qt 5 despite the original standalone `.vcxproj` pointing at Qt 5.12).
- Links `QCustomPlot` (static lib defined at `qCC/extern/QCustomPlot/`; also used by `qG3Point`) for the color-filter histogram widget.

### Known build-environment issue: missing `pwsh.exe`

CC's post-build DLL-copy step invokes `pwsh.exe` (PowerShell 7), which is not installed on this machine. The plugin builds cleanly, but `qTracer.dll` is left at
`C:\CloudCompare\build\plugins\core\Standard\qTracer\Release\qTracer.dll` and is **not** auto-copied next to `CloudCompare.exe`. Three workarounds:

- (recommended) `winget install Microsoft.PowerShell`, reopen cmd, rebuild.
- Use `cmake --install C:\CloudCompare\build --config Release --prefix <dir>` to collect everything.
- Manually copy `qTracer.dll` into `C:\CloudCompare\build\qCC\Release\plugins\`.

vcpkg's `applocal.ps1` also emits red `Add-Content : 資料流是不可讀取的` warnings — harmless (only logging fails).

## Open TODOs

- **Parameter persistence (ColorFilterDlg)** — `PipelineDlg` now persists every parameter + every stage-enabled checkbox via `QSettings`. `ColorFilterDlg` still resets to defaults every run; same pattern would fix it.
- **Progress-bar cancel support** — `TraceClustering` / `PlaneFitting` now drive `ccProgressDialog` via `NormalizedProgress`, but the Cancel button is ignored (return value of `oneStep()` is dropped). Partial results would be safe to return; currently not wired.
- **DB-tree result UX** — stage-3 source cloud is disabled after completion; may want to keep it visible alongside the result group instead.

## Paper terminology (keep UI strings in sync)

Every user-facing string — menu actions, `PipelineDlg` group titles, parameter labels, progress-dialog titles, console messages, DB-tree node names — uses the vocabulary and symbols of the manuscript's §2. Don't rename them casually; a rename here means the paper's figures no longer match the shipped plugin.

| Paper (§2) | Plugin string |
|---|---|
| §2.1 four functionalities | actions `Color Filtering…`, `Outcrop Area Computation…`, `P21 Computation…`, `Fracture Extraction (Pipeline)` |
| §2.2.1 Color Filtering, Eq. (1) `C = aR + bG + cB` | `ColorFilterDlg` header + `Weighting coefficients a, b, c` |
| §2.2.2 Eigenvector Computing, `λ₁≥λ₂≥λ₃`, `v₁`, Eq. (4) `L` | stage 1 title + kernel-radius tooltip |
| §2.2.3 Cylindrical DBSCAN Clustering, Table 2 `ε`, `minPts`, `h = 2ε` | stage 2 title, `Search radius (ε)`, `Minimum points (minPts)`, `Cylinder (along v₁, h = 2ε)`, `Linearity threshold (L)` |
| §2.2.4 Lineation | stage 3 title; output group `<cloud> [trace pieces]` / `Trace piece N` |
| §2.2.5 Trace Clustering, Eqs. (5)-(7) `Δθ_max`, `r_c`, `g_max` | stage 4 title, `Collinearity radius (r_c)`, `Max gap (g_max)`, `Max direction deviation Δθ_max (deg)`; output `Traces` / `Merged trace N` |
| §2.2.6 Plane Fitting, Eqs. (11)-(12) `d_th`, `θ_th`, `L_th` | stage 5 title, `Threshold distance (d_th)`, `Threshold angle θ_th (deg)`, `Threshold length (L_th)`; output `Joint Planes` |
| §2.2.7 Coplanar Plane Merging, Eqs. (13)-(14) `α_max`, `δ_max` | stage 6 title, `Max normal angle α_max (deg)`, `Max plane offset (δ_max)`; output `Merged Joint Planes` |

Two plugin parameters have **no** counterpart in the paper: stage 5's `Max end-point distance` (a fourth pairing criterion the manuscript's §2.2.6 does not list) and stage 6's `Max passes`. Stage 1's `Kernel radius` and all of stage 4's tolerances were symbol-less until §2.2.5 was written.

## Anti-patterns to avoid

- Don't reintroduce separate per-stage `QAction`s / dialogs for the 5 pipeline stages — consolidation into one `PipelineDlg` was a deliberate UX decision. (Note: the `Color Filtering…` action is *not* a pipeline stage — it's a pre-processing tool with live preview that needs its own modal dialog.)
- Don't restore the original hard-coded DBSCAN parameter overrides inside `expandCluster`.
- Don't modify `E:\Research\DFNExtracter\Tracer\` — it's the frozen research reference, not active code.
