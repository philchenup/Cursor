# Why P2P / P2L leave a gap, and why GICP is sensitive to `maxCorrespondenceDistance`

This note is written against [koide3/small_gicp](https://github.com/koide3/small_gicp) and the FilterReg rigid apps in this package. The failure mode in the screenshot — two parallel “ghost” layers of the same geometry, with a consistent gap — is typical of indoor / tabletop scans: large planar patches, sparse points, and a second parallel surface (floor vs table, wall vs wall) a few tens of centimetres away.

## 1. Point-to-point ICP does not snap parallel planes together

P2P (small_gicp `ICPFactor`, FilterReg Kabsch) minimises

\[
E_{\mathrm{P2P}}(T)=\sum_i \|T p_i - q_{\pi(i)}\|^2
\]

with \(\pi(i)\) the nearest neighbour of \(T p_i\) in the target, optionally gated by `maxCorrespondenceDistance`.

On a plane this cost is **almost flat in the two in-plane directions**. Any pair of points on the same plane is an equally good correspondence, so the Gauss–Newton / Kabsch step has:

- a strong component that averages centroids,
- a weak / noisy component along the surface normal.

Consequences:

1. **Sliding.** Without enough non-planar structure (corners, walls, edges) the pose can drift in-plane and never lock.
2. **Wrong identities.** Nearest-neighbour on a sparse plane pairs *some* source point with *some* target point, not with the geometrically corresponding location. The Euclidean residual along the normal is then the distance between two unrelated samples, not the true plane-to-plane gap. The solver therefore under-corrects the offset you see as a gap.
3. **FilterReg GMM mean.** FilterReg does not use NN; it replaces each model point by a permutohedral-lattice GMM mean of nearby observation points. On a plane the mean still lies on (or between) the surface(s) inside \(\sigma\). If \(\sigma\) is larger than the gap, the mean sits *between* the two layers and Kabsch aligns to that phantom mid-surface — a systematic residual gap.

P2P is the wrong metric for the scene in the screenshot. Use it only to get a coarse yaw/centroid, then switch to P2L or GICP.

## 2. Point-to-plane ICP still leaves a gap

Classic P2L (FilterReg `RigidPoint2PlaneTermAssemblerCPU`) minimises

\[
E_{\mathrm{P2L}}(T)=\sum_i \bigl(n_{\pi(i)}^\top (T p_i - q_{\pi(i)})\bigr)^2
\]

which *does* pull along the normal. A remaining gap usually means the **plane you are pulling toward is not the true surface**.

Typical causes, all present in this package / in RGB-D scans:

| Cause | What happens |
| --- | --- |
| **GMM / large correspondence radius mixes two planes** | The target point and its normal are an average of floor + table (or two wall sheets). The implied plane lies in the gap. Gauss–Newton converges to that biased plane. |
| **Fixed FilterReg \(\sigma=8\,\mathrm{cm}\)** | `rigid_pt2pl` never annealed \(\sigma\). After the pose is already close, the kernel still mixes anything within ~8 cm, so the last centimetres never snap. |
| **Unnormalised averaged normals** | Lattice blending of unit normals produces \(\|n\|<1\). Different points get different \(\|n\|\), so the weighted LS is biased. |
| **Noisy / inconsistent normals** | RGB-D curvature and sparse sampling flip or tilt \(n\). The point-to-plane residual is then a projection onto the wrong direction; the step along the true normal is too small. |
| **small_gicp `PointToPlaneICPFactor` is a diagonal approximation** | It uses \((n \odot r)^\top(n \odot r)\) rather than \((n^\top r)^2\). Off-axis components of \(n\) leak into the cost and can stall on thin structures. This package’s `PlaneICP` uses the classic scalar residual. |
| **Degenerate geometry** | A single dominant plane leaves rotation around the normal and in-plane translation weakly observed. A small normal bias becomes a visible parallel offset. |

The screenshot (two parallel sheets, same orientation) is exactly “P2L converged to a biased target plane”, not “P2L cannot move along z”.

## 3. GICP and why `maxCorrespondenceDistance` is so sharp

GICP (Segal et al.; small_gicp `GICPFactor`) models each point as a local Gaussian. The pair cost is the Mahalanobis distance

\[
e_i = r_i^\top \bigl(C^t_{\pi(i)} + R\,C^s_i R^\top\bigr)^{-1} r_i,\qquad r_i = q_{\pi(i)} - T p_i.
\]

Covariances are estimated from k-NN and regularised so the smallest eigenvalue (surface-normal direction) is about \(\varepsilon\lambda_{\max}\) with \(\varepsilon\sim 10^{-3}\) (small_gicp / this package `cov_clip_ratio`). Along the normal the weight is therefore \(\sim 1/\varepsilon \approx 10^3\) times the in-plane weight: **GICP is a very strong plane-to-plane pull**, which is why it can close the gap that P2P cannot.

Correspondence rejection in small_gicp is **not** Mahalanobis. `DistanceRejector` keeps a pair iff

\[
\|T p_i - q_{\pi(i)}\|_2^2 \le d_{\max}^2.
\]

That Euclidean gate is the parameter `maxCorrespondenceDistance`.

### Why a slightly larger \(d_{\max}\) “matches nothing”

1. **Wrong-plane correspondences enter the system.** In the screenshot-like scene the table sits ~0.7 m above the floor. With \(d_{\max}=0.2\,\mathrm{m}\) a table point can only match the table. With \(d_{\max}=1.0\,\mathrm{m}\) a table point’s NN may be the floor (or vice versa), especially after a 5–10 cm residual gap and sparse sampling.

2. **Those wrong pairs dominate GICP.** For two parallel planes the residual \(r\) is almost exactly along the normal, where GICP’s information is \(1/\varepsilon\). One table↔floor pair can outweigh hundreds of correct in-plane pairs. The linear system is then pulled toward a compromise pose, or the LM step diverges. Visually: the clouds “don’t match” / overlap collapses.

3. **Metric mismatch.** The rejector uses Euclidean distance; the optimiser uses Mahalanobis. A pair can pass \(d_{\max}\) and still have a huge \(e_i\). P2P does not have this amplification, so it looks less sensitive to the same \(d_{\max}\) (it just slides). P2L is in between: wrong-plane pairs have residual \(\approx\) inter-plane distance, but they are not up-weighted by \(1/\varepsilon\).

4. **If \(d_{\max}\) is smaller than the current gap**, there are no inliers and GICP does not move at all. So the usable window is

   \[
   \text{current misalignment} \;<\; d_{\max} \;<\; \text{distance to the next parallel surface}.
   \]

   That window is often only a few centimetres on indoor data, which is why the parameter feels binary.

5. **Related knobs that interact with \(d_{\max}\)**  
   - Voxel size: if the voxel is large, NN jumps to the wrong surface more easily.  
   - `cov_knn` / \(\varepsilon\): larger neighbourhoods mix two planes into one covariance; smaller \(\varepsilon\) makes the Mahalanobis blow-up worse.  
   - Initial guess: GICP is a local method. A 7 cm vertical gap with \(d_{\max}=5\,\mathrm{cm}\) is an empty correspondence set.

## 4. What to do (implemented here)

### 4.1 FilterReg `rigid_pt2pl`

- **Renormalise** GMM-averaged normals (`normalize_xyz`).
- **Anneal** the GMM bandwidth from 8 cm down to 4 mm so late iterations cannot mix two surfaces.

### 4.2 Recommended pipeline (`robust_align`, small_gicp-style)

1. **Voxel downsample** both clouds (start at 4× voxel, then 2×, then voxel).
2. **Estimate normals + covariances** on the downsampled cloud; clip \(\lambda_{\min}\).
3. **Coarse-to-fine metric:** P2L on the coarsest level (large Euclidean basin), GICP on the fine levels (snap along normals).
4. **Adaptive \(d_{\max}\):** after each iteration set
   \[
   d_{\max}\leftarrow \max(d_{\min},\;\min(d_{\max},\; 2.5\cdot\mathrm{median}\|r_{\mathrm{inlier}}\|)).
   \]
   Start with a value large enough to see the current gap; the gate then shrinks *before* the next parallel surface can enter.
5. **Cauchy kernel** on the (whitened) residual so a few table↔floor pairs cannot dominate \(H,b\).
6. **Normal-consistency rejector:** drop the pair if \(n_t^\top R n_s < \cos 60^\circ\). Parallel opposite sheets still pass; floor vs a tilted wall does not. Combined with (4) this is the main defence against a large \(d_{\max}\).
7. **Rule of thumb** when you must set a constant:
   - \(d_{\max} \approx 5\text{–}15\times\) voxel,
   - and \(d_{\max} < 0.5 \times\) (nearest parallel-surface spacing),
   - and \(d_{\max} >\) the residual after the previous coarse stage.

```bash
cd filterreg_rigid/build
./robust_align --demo
./robust_align source.pcd target.pcd --method gicp --voxel 0.05 --max-dist 0.5
```

`--demo` builds a floor + L-walls + table at \(z=0.75\,\mathrm{m}\) with a 7 cm vertical gap and prints P2P / P2L / wide GICP / tight GICP / adaptive GICP side by side.
