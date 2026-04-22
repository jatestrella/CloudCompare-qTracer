# qTracer — CloudCompare Plugin

DFN fracture trace extraction pipeline ported into a CloudCompare Standard plugin.

## What this plugin does

Exposes two `QAction`s:

- **Filter by Color…** (optional pre-processing) — opens `ColorFilterDlg` with live 3D preview driven by the cloud's visibility array. Single filter: **C = a·R + b·G + c·B** with a/b/c each ∈ [0, 1] (slider + spinbox). The C min/max threshold is picked on a `HistogramRangeWidget` (QCustomPlot-based histogram of C values with two draggable vertical cursors: green=min, red=max), paired with min/max spinboxes for exact input. To filter by a single channel, zero the other two coefficients (e.g. a=1, b=c=0 → R-only). On OK, `partialClone` creates a new sibling cloud `<name> [color filtered C(a,b,c) min-max]`. Non-destructive: Cancel restores the original visibility.
- **Extract Fractures (Pipeline)** — the main action. Select one point cloud, open `PipelineDlg`, run all 5 stages sequentially:

1. **Compute Eigen** — adds `PC Linearity` + `MaxEigVec_X/Y/Z` scalar fields to the cloud.
2. **DBSCAN** — custom directional variant (cylinder or sphere neighbourhood, eigenvector alignment check); adds `DBSCAN` SF.
3. **Create Traces** — `AutoSegmentationTools::extractConnectedComponents` on the DBSCAN SF → `createTraces` produces `ccFacet`s + trace polylines in a `[facets]` group.
4. **Trace Clustering** — region-growing merge of close-collinear traces → `Traces` group.
5. **Plane Fitting** — pairs near-intersecting combined traces into `ccFacet` joint planes → `Fit Joint Planes` group.

All three stage-outputs are added under one parent DB node `qTracer [<cloud name>]`.

## Provenance

Ported from the standalone console app at `E:\Research\DFNExtracter\Tracer`. The original project is kept as a frozen research reference — **don't modify it**. Backup at GitHub `CC-qtracer` (private).

## Non-obvious decisions made during the port

These changes are intentional — don't silently undo them:

- `GetCloudLinearity` — original was un-compilable (`Linearity` scope bug, condition reversed, division by a1-when-all-zero). Fixed.
- `DBSCANParams` struct — the original `expandCluster` hard-coded `radius=0.03`, `minPoints=50`, `maxAngleDeg=15`, `linearityThreshold=0.9`, `searchType=1` at the top of the function, **overriding the `radius` argument**. Now all 5 flow through `PipelineDlg` via the struct. The override lines were removed so the dialog actually controls behavior.
- `ComputeEigen` no longer calls `runDBSCAN` at its end — each stage is invoked explicitly from `qTracer.cpp::doPipeline()`.
- `<ppl.h>` removed (was an unused include).
- At the start of stage 3 the `DBSCAN` SF is deleted from the source cloud so `partialClone` doesn't copy it into every sub-cloud. Same behavior as the standalone source.

Algorithm logic (distance tests, `InCone` check, eigenvector searches, `extractConnectedComponents` flow, facet + polyline construction) is preserved verbatim.

## File map

- `include/qTracer.h` / `src/qTracer.cpp` — plugin class, two actions (`Filter by Color…` + `Extract Fractures (Pipeline)`), selection gating, `doColorFilter()` + `doPipeline()`.
- `include/ColorFilterDlg.h` / `src/ColorFilterDlg.cpp` / `ui/ColorFilterDlg.ui` — interactive color-filter dialog. Uses the cloud's visibility array + `redrawDisplay()` for live preview; 75 ms debounce via `QTimer`; reject() restores the pre-dialog visibility state. Histogram is debounced separately (120 ms) and rebuilt only when a/b/c change.
- `include/HistogramRangeWidget.h` / `src/HistogramRangeWidget.cpp` — `QCustomPlot` subclass: histogram bars + two draggable vertical cursors (green/red) for a min/max range picker. Emits `lowerChanged(double)` / `upperChanged(double)`. Registered in `ColorFilterDlg.ui` as a promoted widget (`<customwidgets>`). Linked via `target_link_libraries(qTracer QCustomPlot Qt6::PrintSupport)` in the plugin root CMakeLists.
- `include/PipelineDlg.h` / `src/PipelineDlg.cpp` / `ui/PipelineDlg.ui` — combined parameter dialog (5 `QGroupBox` sections, one per stage).
- `include/IdentifyFracture.h` / `src/IdentifyFracture.cpp` — algorithm API (`ComputeEigen`, `runDBSCAN`, `expandCluster`, `createTraces`, `TraceClustering`, `PlaneFitting`, helpers). All 5 pipeline entry points (`ComputeEigen`, `runDBSCAN`, `createTraces`, `TraceClustering`, `PlaneFitting`) accept an optional `GenericProgressCallback*` and drive it via `NormalizedProgress`. Note: stage 3's `extractConnectedComponents` is a CCCoreLib built-in that does not itself take a progress callback — progress during stage 3 only tracks the facet-build loop.
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

- **Parameter persistence** — `PipelineDlg` and `ColorFilterDlg` reset to defaults every run; would save/load via `QSettings` under an organization key.
- **Progress-bar cancel support** — `TraceClustering` / `PlaneFitting` now drive `ccProgressDialog` via `NormalizedProgress`, but the Cancel button is ignored (return value of `oneStep()` is dropped). Partial results would be safe to return; currently not wired.
- **DB-tree result UX** — stage-3 source cloud is disabled after completion; may want to keep it visible alongside the result group instead.

## Anti-patterns to avoid

- Don't reintroduce separate per-stage `QAction`s / dialogs for the 5 pipeline stages — consolidation into one `PipelineDlg` was a deliberate UX decision. (Note: the `Filter by Color…` action is *not* a pipeline stage — it's a pre-processing tool with live preview that needs its own modal dialog.)
- Don't restore the original hard-coded DBSCAN parameter overrides inside `expandCluster`.
- Don't modify `E:\Research\DFNExtracter\Tracer\` — it's the frozen research reference, not active code.
