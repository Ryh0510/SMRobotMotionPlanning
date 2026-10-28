# Workpiece virtual baffle

Enabling the collision module, including Python `set_collision_enabled(True)`, prepares and draws a virtual baffle for every collision-enabled workpiece whose geometry supports it. No extra user detector pair is installed or substituted. Offline planning prepares the same constraint in its isolated scene.

## Construction and units

Reuse `BoundaryDetector::detectBoundaries` in its topology-diagnostic mode. It must return exactly two closed, unbranched rims and no nonmanifold edges. Rims are unordered: there is no upper/lower or world-axis test. Independently resample both closed polylines by arclength (256 samples), search both windings and cyclic phases using centered, size-normalized geometric discrepancy, and fit the paired midpoint cloud by the smallest covariance eigenvector. Reject a degenerate fit, ambiguous correspondence, or excessive maximum residual (default 5 mm; residual above slab thickness is explicitly reported as a fit warning). Original sampling indices are never paired directly.

Mesh and baffle calculations use metres, as the native scene does. Interface/configuration distances are mm; default slab thickness is 1 mm. Uniform collision scale is applied before fitting; the workpiece rigid transform is applied at query and display time. Reuse `UniformSlicer::traceContoursFromIntersections` for mesh-plane sections, including its endpoint welding, duplicate/coplanar-edge handling and connected contour tracing. Multiple closed sections use even/odd fill, including nested holes. Open chains and branched/invalid components are diagnosed; no fictitious closing segments are inserted. Missing, extra, branched, nonplanar or ambiguous boundaries/sections disable that workpiece baffle with a reason.

## Collision and presentation

The forbidden solid is the infinite slab outside the planar aperture. Robot/tool triangles are clipped against both slab faces. Their planar projections are checked for forbidden vertices and aperture-edge intersection, including concavity and enclosed hole boundaries. Solid containment additionally checks aperture boundary vertices inside a body. A transformed bounding box is a broadphase only. Triangle meshes and boxes use their collision surface; round analytic primitives currently use explicitly conservative circumscribed boxes. This can reject additional states but cannot permit a primitive through a forbidden area. Missing/unsupported physical body geometry fails the query closed rather than silently dropping a link.

Every enabled link collision element and installed attachment is checked, independently of selected ordinary collision pairs. Every sampled node/edge state goes through this check; the evaluator additionally enforces requested clearance by conservative baffle inflation. Existing motion sampling resolution is unchanged: this is not a claim of strict continuous collision detection.

The rendered orange translucent window is finite for readability; collision is analytic and has no outer edge. Geometry is clipped from the aperture using scanline trapezoids. Visibility does not disable the constraint. Baffles follow their workpiece transforms. `rws.virtual_baffles` is a project extension containing `enabled`, `visible`, `thickness_mm`, and `fit_tolerance_mm`; absent settings default to automatic activation. Asset timestamp/size, scale and geometry parameters invalidate the construction cache. Existing saved projects receive these defaults on save. The model asset remains the source of truth and sections are rebuilt on load.

SDK: `client.virtual_baffles()` returns per-workpiece topology, fit residuals, enabled state, diagnostics and current collision. `query_collision` retains pair counts and reports `selected_pairs_in_collision` separately from the additional `virtual_baffle_collision`. No nearest/penetration distance is fabricated for the baffle; SDK distance is null when this additional constraint causes collision. Native ordinary-scene distance feedback remains unchanged.

Read-only CLI diagnostics (does not create a solve):

```
RwsTrajectoryEvaluate.exe scene.sys.json --baffle-diagnostics report.json
```

## Exports

New LS exports use CNT100 for every move, including the initial joint move. MOD exports use z100 for every move. This applies to partial web exports, native slicing exports and native Cartesian RAPID exports. Point-type annotations, incomplete status and unresolved-point reports are retained. Blending syntax does not certify the controller swept path. Historical exported artifacts are not silently rewritten.
