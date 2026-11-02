# G82 - Visual docking of a TurtleBot3 with an EKF and LQR (Track A)

**42043 Robotics Studio 1, Integrated Project, Spring 2026, Group 82**

> TurtleBot3 Waffle Pi in **MuJoCo** maps a room with obstacles using its **2D LiDAR**, tracks the path with a **gain-scheduled LQR**, and docks to an **ArUco-marked charging station**.
> All decisions use the pose from an **EKF** that fuses **wheel-encoder odometry** with **range, bearing and
> relative heading** to ArUco landmarks. Ground truth is used for evaluation only.

|Demo videos|Track|Platform|
|-|-|-|
|[Presentation demo video](https://youtu.be/OJ8svSIu7EU) (about 5 min, comparison of the runs); individual clips in [`evidence/videos/`](evidence/videos/)|A, mobile robot visual waypoint navigation|MuJoCo 3, TurtleBot3 Waffle Pi (ROBOTIS menagerie)|

[![Presentation demo video: comparison of the runs](https://img.youtube.com/vi/OJ8svSIu7EU/hqdefault.jpg)](https://youtu.be/OJ8svSIu7EU)



*Presentation demo video (click to watch on YouTube): the runs are shown together as a comparison. The separate clips it is made from are in* [*`evidence/videos/`*](evidence/videos/)*.*



The per-seed results behind the evidence clips are in section 7a.



![result](docs/figures/map_nominal.png)

---

## 1. Quick start

```bash
git clone https://github.com/NesaraSuresh/G82-Turtlebot && cd G82-Turtlebot
python -m venv .venv && source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python main.py                         # full pipeline, nominal scenario -> results/nominal_seed1_ekf/
python main.py --viewer                # same, with the live MuJoCo viewer
python main.py --scenario push --record   # disturbed trial + demo video (results/.../demo.mp4)
python live_camera.py --scenario nominal --seed 1   # live robot-camera window: dock target green, wall landmarks orange
```

Each run writes `metrics.json`, `log.npz`, `map.png`, `estimation.png` and `control.png` (plus `demo.mp4` with `--record`).


**Reproduce every number on this page** (about 1 h on a laptop, mostly the Monte-Carlo runs):

```bash
python scripts/calibrate_camera.py      # B: checkerboard calibration -> config/camera_calibration.json
python scripts/characterise_noise.py    # B/C: identify R (camera) and Q (odometry) -> config/noise_params.json
python -m src.analysis.model_analysis   # A: controllability / observability / LQR poles
python scripts/run_trials.py --scenarios nominal noisy slip push occlusion stress odom_only --seeds 10 --name robustness
python scripts/run_trials.py --scenarios nominal --seeds 10 --sweep ekf.r_scale=1,10,50 --name r_sweep
python scripts/run_trials.py --scenarios nominal --seeds 10 --sweep "lqr.u_max=(0.05,0.3),(0.1,0.6),(0.2,1.2)" --name lqr_sweep
python scripts/run_trials.py --scenarios nominal stress --seeds 10 --estimators ekf ukf --name ekf_vs_ukf
python -m pytest -q                     # unit tests, including "no ground truth in decision code"
```

Scenarios (`src/config.py`): `nominal`, `noisy` (3x sensor noise), `slip` (wrong wheel radii and base in odometry),
`push` (25 N lateral shove for 0.25 s at t = 12 s), `occlusion` (camera blacked out 5 to 15 s), `stress` (all of the above
plus one step of command latency), `odom_only` (camera disabled, as an ablation).
Every seed randomises the obstacle layout and the true start pose (the robot only knows the nominal start).

---

## 2. System overview

```mermaid
flowchart LR
  subgraph SIM["MuJoCo (src/sim) via HAL (src/hal)"]
    ENC[Wheel encoders<br/>4096 ticks/rev]
    CAM[RGB camera 640x480<br/>48.8 deg vFoV]
    LID[2D LiDAR<br/>360 rays, 3.5 m]
    ACT[Wheel velocity servos]
  end
  ENC -->|ticks| ODO[Odometry<br/>v, w]
  ODO -->|predict| EKF[EKF<br/>x, y, theta, P]
  CAM --> ARU[ArUco + PnP<br/>T_B_M -> r, b, alpha]
  ARU -->|gated update| EKF
  LID --> OG[Occupancy grid<br/>at EKF pose]
  EKF --> OG
  OG --> AST[A* + smoothing<br/>reference path]
  EKF --> MIS[Mission FSM<br/>NAVIGATE / ALIGN / DOCK]
  AST --> MIS
  LID -->|front sector| MIS
  MIS --> LQR[Gain-scheduled LQR]
  LQR -->|v, w| ACT
  GT[(Ground-truth oracle)] -.evaluation only.-> LOG[Logger / metrics / video]
  EKF -.-> LOG
```

Rates: control, odometry and EKF predict at 20 Hz; camera and LiDAR at 10 Hz; physics at 500 Hz.
The controller, planner and estimator only see the `RobotInterface` (`src/hal/interface.py`), which has no
"true pose" method. Moving to the real robot means replacing `src/sim/mujoco_robot.py` with a ROS 2 driver that
implements the same five methods.

### Frames

```mermaid
flowchart LR
  W["W world<br/>room centre, z up"] -->|"EKF state (x, y, theta)"| B["B base_link<br/>mid-axle, x forward"]
  B -->|"fixed: t = (0.0865, 0, 0.107)<br/>R = [[0,0,1],[-1,0,0],[0,-1,0]]"| C["C camera_optical<br/>z forward, x right, y down"]
  C -->|"solvePnP (IPPE_SQUARE)"| M["M marker<br/>x right, y up, z out"]
  W -->|"known map: pose of every marker"| M
  B -->|"fixed: t = (-0.049, 0, 0.172)"| L["L lidar"]
```

Measurement chain: pixels → `solvePnP` gives `T_C_M` → `T_B_M = T_B_C · T_C_M` → `z = [‖p_xy‖, atan2(p_y, p_x), atan2(n_y, n_x)]`,
where `p` is the marker centre and `n` the marker normal (its z axis), both in B.
The EKF predicts the same quantities from `(x, y, θ)` and the marker's map pose `T_W_M`.
MuJoCo's camera frame (x right, y up, z backwards) is never used downstream. Using it instead of the optical frame
is the classic bug, and `scripts/calibrate_camera.py` checks `T_B_C` numerically (section 4).

---

## 3. Modelling (A)

**Platform model.** Differential drive, wheel radius r = 0.033 m and wheel separation L = 0.288 m (taken from the MJCF).

* Inputs: `u = [v, ω]`, with `v = r(ω_R + ω_L)/2` and `ω = r(ω_R − ω_L)/L`.
* State: `x = [x, y, θ]ᵀ`, with `ẋ = v cos θ`, `ẏ = v sin θ`, `θ̇ = ω`.

The estimator uses the mid-point discretisation, with Jacobians F and G in `src/estimation/ekf.py`.

**Assumptions:**

* No lateral slip (non-holonomic).
* Flat floor, so the model is planar (2D).
* Wheel-speed dynamics are much faster than the 20 Hz loop, so v and ω are treated as inputs.
* Actuator saturation is handled by preserving curvature (`MujocoTurtlebot.send_velocity`).

The physics deliberately violate the no-slip assumption: there is 3 to 5 % slip in arcs, which odometry cannot see.

**Changes to the ROBOTIS model**, all marked `G82` in the XML:

* **Velocity servo gain raised.** The stock kv = 0.1 tracked only about 50 % of the commanded wheel speed.
* **Base collision replaced by a box.** The stock mesh hull dragged on the floor.
* **Wheels made thin ellipsoids instead of cylinders.** A flat cylinder touches the floor at both rim edges and scrubbed about 20 % of the yaw rate in arcs, found in the odometry identification below.

**Controllability** (`python -m src.analysis.model_analysis`, full output in `docs/model_analysis.md`):

|model|rank C|controllable|
|-|-|-|
|unicycle linearised at rest (A = 0)|2|**no**: no instantaneous sideways motion|
|tracking error, v_r = 0, ω_r = 0|2|**no**|
|tracking error, v_r ≠ 0 or ω_r ≠ 0|3|yes|

The LQR is therefore designed on the **tracking-error model** about the moving reference, and its gain is scheduled
on `(v_r, ω_r)` with |v_r| ≥ 0.03 m/s (section 6). Pure rotation (`ALIGN` states) uses a separate heading loop.

**Observability** (local observability matrix `O = [H; HF; HF²]`, evaluated numerically):

|sensors|rank O|unobservable direction|
|-|-|-|
|odometry only|0|everything: drift is unbounded|
|1 marker, range + bearing, static **or moving**|2|rotation of the pose about the marker|
|1 marker, range + bearing + relative heading α|3|none|
|2 markers, range + bearing|3|none|

A key point: **motion does not fix the single-landmark case**. Odometry is body-frame, so rotating the whole pose
about the landmark changes no future measurement. That is why the camera measurement includes the PnP relative
heading α (when the marker is large enough, see section 4), and why the room has six wall landmarks besides the dock.

---

## 4. Sensing (B)

**Camera calibration** (`scripts/calibrate_camera.py`): a 9×6 checkerboard (25 mm squares) is placed at 60 random
poses in front of the robot camera, rendered with the image-noise model, then put through `cv2.calibrateCamera`
(k3 and tangential terms fixed to zero).

|parameter|calibrated|analytic (MuJoCo fovy=48.8 deg)|difference|
|-|-|-|-|
|fx [px]|527.47|529.08|-0.30 %|
|fy [px]|527.02|529.08|-0.39 %|
|cx [px]|318.98|319.50|-0.52 px|
|cy [px]|241.07|239.50|+1.57 px|
|k1, k2|-0.0228, +0.0488|0, 0 (ideal pinhole)||

Fixing the tangential terms mattered: with them free, cx drifted by 6 px, a 0.7° bearing bias, because they trade off against the principal point.
The **extrinsic `T_B_C` check** reprojects the true board pose through `T_B_C` and compares it with PnP. The board centre agrees
to 2.77 mm (max 4.41 mm, 60 views) on average, which confirms the frame chain.

**Sensor models and noise** (MuJoCo is noise-free, so all noise is injected in `src/sim/mujoco_robot.py`):

|sensor|model|
|-|-|
|encoders|true wheel angle + random-walk slip (σ = 0.002 rad/step while moving), quantised to 4096 ticks/rev|
|camera|render → 3×3 Gaussian blur → N(0, 4) grey-level noise; optional blackout (occlusion)|
|LiDAR|`mj_multiRay` from the scan plane, N(0, 1 cm) range noise, 2 % dropouts, 0.12 to 3.5 m|

**Noise identification** (`scripts/characterise_noise.py`). This is an offline experiment that uses ground truth, as use motion capture would be used on the real robot.

* **Camera R.** The robot is placed at about 800 poses relative to the 12 cm dock marker and a 20 cm wall marker:

  * range 0.3 to 4 m
  * viewing angle ±60°
  * image offset ±15°

  The errors of `[r, b, α]` are binned by the marker's **apparent size ρ = range / marker size**, and a robust (MAD) σ is fitted per bin.



![camera noise](docs/figures/camera_noise.png)

* Range error grows with ρ², while bearing error is flat at about 0.0014 rad (sub-pixel).
* α (planar-PnP orientation) has an **ambiguity**: above ρ ≈ 15.3, more than 5 % of α estimates are flipped by tens of degrees.
The EKF therefore uses α only for ρ < 15.3.
* **Odometry Q.** The robot drives 19 constant (v, ω) commands, and odometry is compared with truth over 1 s windows.
Per-step differences of MuJoCo's pose contain contact jitter that averages out, so they overestimate the noise.
The error is almost entirely systematic slip (3 to 5 % in arcs, about 1 % straight), which gives σ_v = 0.032|v| and σ_ω = 0.200|ω|.



![odometry noise](docs/figures/odometry_noise.png)

**Noise handling in the pipeline:**

* Gaussian-blur-robust ArUco detection with sub-pixel corners.
* Detections are rejected when the PnP reprojection error exceeds 2 px or the marker normal points away from the robot.
* A χ² innovation gate is applied in the EKF.
* LiDAR is fused into a log-odds grid, so single noisy returns do not create obstacles.

---

## 5. Estimation (C)

EKF in `src/estimation/ekf.py`. A UKF with the same models is in `src/estimation/ukf.py`.

|item|design|value / source|
|-|-|-|
|state|`[x, y, θ]` in W|-|
|predict|odometry `(v, ω)` at 20 Hz, mid-point model|-|
|Q|input-noise model `M = diag((a1|v|
|measurement|`z = [r, b, α]` per visible marker (α only if ρ < 15.3)|known marker map|
|R|`diag(σ_r(ρ)², σ_b(ρ)², σ_α(ρ)²) · r_scale`|identified models, r_scale = 10 (see below)|
|gating|NIS χ² test, p = 0.997 (2 or 3 dof)|-|
|update|Joseph form|-|
|lock-out recovery|after 8 consecutive gate rejections, inflate P by diag(0.1², 0.1², 0.08²)|-|
|initialisation|nominal start pose, P₀ = diag(0.15², 0.15², 0.10²)|true start is nominal + N(0, (0.1, 0.1, 0.08))|

**Choosing R: consistency vs responsiveness.** The R identified from *static* images is too optimistic once the robot moves,
because body rocking, slip and timing are not in that experiment. The R scale was therefore chosen with a sweep on paired seeds
(`python scripts/run_trials.py --sweep ekf.r_scale=1,10,50`):

|condition|scenario|estimator|n|success_rate|dock_pos_err|dock_yaw_err_deg|est_rmse_xy|nees_mean|track_e2_rmse|cross_track_rmse|effort|duration|
|-|-|-|-|-|-|-|-|-|-|-|-|-|
|ekf.r_scale=1|nominal|ekf|10|100 %|2.02 ± 0.98 cm|2.06 ± 1.24|8.60 ± 4.18 cm|3.78 ± 1.56|3.01 ± 1.75 cm|7.95 ± 5.25 cm|0.38 ± 0.03|40.69 ± 3.64|
|ekf.r_scale=10|nominal|ekf|10|100 %|2.04 ± 1.01 cm|2.16 ± 1.31|8.71 ± 4.04 cm|1.93 ± 1.04|2.73 ± 1.82 cm|7.89 ± 5.34 cm|0.38 ± 0.02|40.72 ± 3.53|
|ekf.r_scale=50|nominal|ekf|10|100 %|2.08 ± 0.97 cm|2.24 ± 1.26|8.86 ± 3.96 cm|1.58 ± 0.72|2.56 ± 1.86 cm|7.92 ± 5.36 cm|0.38 ± 0.02|41.03 ± 4.02|

* **r_scale = 1:** the filter is overconfident (mean NEES ≈ 3.8 against the ideal 3 for 3 dof).
Overconfidence is what caused the gate lock-out failure (section 7).
* **r_scale = 50:** under-confident, with no accuracy gain.
* **Chosen:** r_scale = 10, which is slightly conservative (NEES < 3). Docking error and estimation RMSE barely change across
the sweep, so the choice trades only covariance realism, not accuracy. Log-interpolating the sweep, r_scale ≈ 3 would give NEES ≈ 3.
We stayed conservative because a too-small R fails catastrophically through the gate, while a too-large R only costs a little responsiveness.
* *Caveat:* the mean NEES includes the start of each run, when P₀ is deliberately generous, which pulls it below 3.



![estimation](docs/figures/estimation_nominal.png)

*Estimate minus truth with ±3σ bounds. Ticks show when a marker is visible. The covariance grows at the start because
no marker is within 3.5 m of the start pose, then collapses at the first landmark update.*

---

## 6. Control (D)

**Tracking error in the robot frame:**

* `e₁ = cos θ (x_r − x) + sin θ (y_r − y)` (along-track)
* `e₂ = −sin θ (x_r − x) + cos θ (y_r − y)` (cross-track)
* `e₃ = θ_r − θ` (heading)

The inputs are `u₁ = v_r cos e₃ − v` and `u₂ = ω_r − ω`. Linearised:

`ė = [[0, ω_r, 0], [−ω_r, 0, v_r], [0, 0, 0]] e + [[1, 0], [0, 0], [0, 1]] u`

* **Discretisation:** at 20 Hz. The DARE is solved every step for the current `(v_r, ω_r)`, which gives a gain-scheduled LQR.
* **Command:** `v = v_r cos e₃ + K₁e` and `ω = ω_r + K₂e`. The feed-forward `(v_r, ω_r = v_r κ)` comes from the smoothed A\* path.
* **Weights (Bryson's rule):**

  * Errors: max along-track 0.10 m, cross-track 0.05 m, heading 0.10 rad, so Q = diag(100, 400, 100).
  * Inputs: max corrections 0.10 m/s and 0.6 rad/s, so R = diag(100, 2.8).
  * Cross-track gets the largest weight because it is what collides with obstacles.
* **Speed profile:** `v_r(s) = min(0.18, √(a_lat/|κ|), √(2 a_long (s_end − s)))`. Docking speed is proportional to the remaining distance, from 0.08 m/s down to 0.025 m/s.

|v_r|w_r|K (2x3)|closed-loop eigenvalue magnitude (discrete)|continuous-equivalent poles [1/s]|
|-|-|-|-|-|
|0.03|0.0|[[ 0.98  0.    0.  ] [ 0.   10.32  5.23]]|0.951, 0.742, 0.997|-1.00+0.00j, -5.98+0.00j, -0.06+0.00j|
|0.08|0.0|[[ 0.98  0.    0.  ] [ 0.   10.29  5.32]]|0.951, 0.742, 0.992|-1.00+0.00j, -5.98+0.00j, -0.16+0.00j|
|0.18|0.0|[[ 0.98 -0.   -0.  ] [-0.   10.24  5.52]]|0.951, 0.742, 0.982|-1.00+0.00j, -5.97+0.00j, -0.36+0.00j|
|0.18|0.5|[[ 1.49 -1.24 -0.05] [-1.38  4.95  5.34]]|0.742, 0.959, 0.959|-5.97+0.00j, -0.84+0.63j, -0.84-0.63j|

**What limits the gains:**

* The slowest closed-loop mode is the cross-track/heading coupling. Its pole scales with v_r, because cross-track error
can only be removed by driving with a heading offset. This is the controllability result above, seen in the poles, and it is why the gain uses v_r ≥ 0.03 m/s.
* The ω limit of 1.82 rad/s saturates on replans, visible as spikes in the control plot.

**Control effort vs tracking error** (sweep of the Bryson input limits, paired seeds):

|condition|scenario|estimator|n|success_rate|dock_pos_err|dock_yaw_err_deg|est_rmse_xy|nees_mean|track_e2_rmse|cross_track_rmse|effort|duration|
|-|-|-|-|-|-|-|-|-|-|-|-|-|
|lqr.u_max=(0.05,0.3)|nominal|ekf|10|100 %|2.11 ± 1.06 cm|2.23 ± 1.37|8.73 ± 4.04 cm|1.96 ± 1.01|2.90 ± 1.91 cm|7.95 ± 5.40 cm|0.37 ± 0.02|41.13 ± 3.60|
|lqr.u_max=(0.1,0.6)|nominal|ekf|10|100 %|2.04 ± 1.01 cm|2.16 ± 1.31|8.71 ± 4.04 cm|1.93 ± 1.04|2.73 ± 1.82 cm|7.89 ± 5.34 cm|0.38 ± 0.02|40.72 ± 3.53|
|lqr.u_max=(0.2,1.2)|nominal|ekf|10|100 %|1.99 ± 0.97 cm|2.08 ± 1.28|8.85 ± 4.17 cm|1.92 ± 0.86|2.82 ± 1.87 cm|7.95 ± 5.48 cm|0.39 ± 0.03|40.48 ± 3.62|

The trade-off is weak in this task:

* Most of the command is feed-forward (v_r, ω_r = v_r κ), so the feedback term contributes little to effort.
* The dominant tracking error early in each run is *estimation* error before the first marker is seen, which no controller gain can remove.
It shows in the true cross-track error but not in the controller's own error e₂.
* Across a 16× range of input limits, docking error is unchanged.
* The cheapest setting gives slightly larger controller cross-track error (e₂ about 5.0 vs 4.7 cm).
* The most aggressive setting uses slightly more effort (0.43 vs 0.40).
* The middle setting is kept: it sits at the knee of these trade-offs.

**Docking accuracy is control-limited, not estimation-limited:**

* At the dock, the estimate is within a few millimetres and a fraction of a degree of the truth (2 mm in `nominal`, seed 1).
* The residual of about 2 cm and 2° comes from the slow cross-track mode at docking speed (0.025 to 0.08 m/s), combined with the
up to 12 cm lateral offset allowed at the pre-dock pose.



![control](docs/figures/control_nominal.png)

*Commands vs feed-forward reference, and tracking error. The true distance to the path grows at the start because the robot
follows its estimate, which is wrong until the first landmark is seen.*

---

## 7. Integration and robustness (E)

**Mission:** NAVIGATE → (ALIGN_PATH) → ALIGN_DOCK → DOCK → DOCKED.

* **NAVIGATE:** A\* is planned on the LiDAR occupancy grid, which is built at the EKF pose. It replans when the path becomes
blocked, when the estimate deviates more than 0.35 m, or when the path ends.
* **LiDAR safety layer:** blocks forward motion if any return in ±35° is within 12 cm of the robot front.
* **ALIGN_DOCK:** rotates to face the dock at the pre-dock pose, 0.85 m from the marker.
* **DOCK:** a straight-line LQR approach to 0.35 m from the marker, where the robot front stops on the yellow charging pad.
* **Success** (ground truth): final position within 5 cm and yaw within 10° of the dock pose.

**Monte-Carlo results:** 10 random worlds per scenario. Each seed randomises the obstacles and the true start, and
seeds are paired across scenarios. Mean ± std, from `docs/results/robustness.md`.

|condition|scenario|estimator|n|success_rate|dock_pos_err|dock_yaw_err_deg|est_rmse_xy|nees_mean|track_e2_rmse|cross_track_rmse|effort|duration|
|-|-|-|-|-|-|-|-|-|-|-|-|-|
|default|noisy|ekf|10|100 %|2.03 ± 1.01 cm|2.21 ± 1.36|9.00 ± 4.39 cm|2.24 ± 1.10|2.78 ± 1.98 cm|8.01 ± 5.93 cm|0.38 ± 0.03|41.27 ± 3.67|
|default|nominal|ekf|10|100 %|2.04 ± 1.01 cm|2.16 ± 1.31|8.71 ± 4.04 cm|1.93 ± 1.04|2.73 ± 1.82 cm|7.89 ± 5.34 cm|0.38 ± 0.02|40.72 ± 3.53|
|default|occlusion|ekf|10|100 %|1.96 ± 1.00 cm|1.98 ± 1.31|10.16 ± 5.31 cm|1.82 ± 0.78|3.24 ± 2.64 cm|9.56 ± 6.83 cm|0.38 ± 0.03|40.93 ± 3.92|
|default|odom_only|ekf|10|0 %|38.47 ± 28.41 cm|4.90 ± 3.77|29.80 ± 19.21 cm|2.16 ± 1.45|0.47 ± 0.29 cm|21.73 ± 15.13 cm|0.37 ± 0.03|40.54 ± 4.27|
|default|push|ekf|10|100 %|1.93 ± 1.05 cm|1.99 ± 1.39|8.84 ± 3.75 cm|11.40 ± 8.76|3.52 ± 1.70 cm|8.18 ± 4.90 cm|0.39 ± 0.03|41.26 ± 3.76|
|default|slip|ekf|10|90 %|2.63 ± 1.56 cm|2.81 ± 1.76|11.01 ± 4.26 cm|25.88 ± 10.83|5.04 ± 1.36 cm|11.21 ± 4.78 cm|0.41 ± 0.01|41.97 ± 6.40|
|default|stress|ekf|10|100 %|2.63 ± 1.25 cm|2.69 ± 1.50|9.85 ± 3.40 cm|12.54 ± 7.40|4.13 ± 1.58 cm|8.91 ± 4.37 cm|0.41 ± 0.03|40.20 ± 4.52|

**Reading the table:**

* **Columns:**

  * `dock_pos_err` and `dock_yaw_err_deg` are measured against ground truth.
  * `est_rmse_xy` is the EKF error over the whole run, dominated by the first seconds before any marker is in range.
  * `track_e2_rmse` is the controller's own cross-track error relative to its estimate.
  * `cross_track_rmse` is the *true* distance to the planned path, so it includes estimation error.
* **`odom_only` is the observability result in practice.** The controller tracks its estimate almost perfectly (e₂ RMS < 1 cm),
yet the robot misses the dock by about 40 cm, because without exteroception the estimate drifts.
Good control on a bad estimate is still a failure, which is why the brief forbids ground truth in the loop.
* **`occlusion`:** 10 s of camera blackout raises the estimation RMSE, but the run still docks. Odometry holds the pose to a few
centimetres over 10 s, and the covariance grows honestly (NEES stays near 2).
* **`push`:** succeeds, but the NEES spikes: the shove is not in the process model.
* **`slip` and `stress`:** high NEES (inconsistent) despite successful docking. A biased odometry model makes the filter
overconfident between marker sightings (see failure 6).



**Failures and diagnosis** (what went wrong during development and how each was traced):

1. **Filter lock-out after a bad PnP heading (estimator).**

   * *Symptom:* in an early `push` trial, the dock was missed by 30 cm. NEES jumped to about 30 and stayed there, while markers were visible the whole time.
   * *Cause:* a dock-marker reading at 2.3 m (ρ ≈ 19) had an α error of −7° (planar-PnP ambiguity). The filter accepted it with
σ_α = 0.05 rad and became confidently wrong, and the χ² gate then rejected every correct measurement.
   * *Fix:*

     * σ_α now grows with ρ.
     * α is used only for ρ < 15.3.
     * Covariance is inflated after 8 consecutive rejections.
   * *Result:* the same trial now docks within 1.4 cm.
2. **Odometry 20 % yaw error in arcs (model / sim).** Found by the odometry identification: flat cylinder wheels scrub at
two contact points. Thin ellipsoid wheels (single contact) reduced this to 3 to 5 %, which matches the real robot's typical slip.
3. **Velocity servo too soft (model).** With the stock kv = 0.1, the wheels reached only about 50 % of the commanded speed, so the
LQR feed-forward was meaningless. kv was raised to 1.5, representing the Dynamixel's internal velocity loop.
4. **Overconfident R from static characterisation (estimator).** NEES was about 10. Fixed with the R sweep above.
5. **Sawtooth in the speed command (control).**

   * *Symptom:* the control plot showed v oscillating ±0.02 m/s at about 2 Hz.
   * *Cause:* the reference was the nearest path *vertex* (5 cm spacing), so the along-track error jumped by up to 5 cm and the LQR chased it.
   * *Fix:* continuous projection onto the path segment with interpolated (x, y, θ, v, κ).
6. **`slip` scenario is inconsistent (open issue).** The robot still docks, but NEES is well above 3, because a wrong wheel
radius is a *bias* and the EKF models only zero-mean noise. A justified extension is to augment the state with odometry scale
factors (r_L, r_R, L) and estimate them online. Landmarks make them observable while turning.

**Limits of the approach** (what the results do and do not support):

* The marker map is assumed known and exact. Map errors appear directly as pose bias.
* The pose is planar (3 DOF), so body pitch while accelerating appears as range and bearing noise. That is part of why R had to be inflated.
* Simulated camera images have no motion blur, rolling shutter or lighting change. Real detection ranges will be shorter, especially for the 12 cm dock marker.
* The occupancy grid is built from the estimated pose, so large estimation errors corrupt the map. There is no loop closure.
* Success rates come from 10 seeds per scenario, so they carry roughly ±31 % binomial uncertainty.

### 7a. Selected runs: nominal vs push, odometry only, and uncertainty plots

The Monte-Carlo table above gives statistics. This section looks at individual seeds in detail, with video, figures and the raw `metrics.json` of each run kept in [`evidence/`](evidence/) (`videos/`, `figures/`, `metrics/` and `results_table.txt`). Because the seed fixes the obstacle layout, a disturbed run is only ever compared with the nominal run of the **same seed**.

**Five seeds, nominal vs push** (the summary table is printed by `python summarise.py`):

|scenario|seed|docked|dock error (cm)|dock yaw (deg)|est. RMSE (cm)|NEES|recoveries|
|-|-|-|-|-|-|-|-|
|nominal|1|yes|0.7|0.44|5.2|1.0|0|
|nominal|2|yes|1.7|1.77|6.3|1.2|0|
|nominal|3|yes|3.4|3.86|6.8|4.0|0|
|nominal|4|yes|1.4|1.40|4.9|3.6|0|
|nominal|5|yes|0.7|0.24|11.3|1.1|0|
|push|1|yes|0.8|0.62|8.9|2.8|0|
|push|2|yes|1.8|2.04|8.4|1.9|0|
|push|3|yes|3.7|4.16|7.0|29.2|1|
|push|4|yes|1.6|1.70|6.6|11.4|1|
|push|5|yes|0.8|0.65|12.0|2.4|0|

* All ten runs dock. Means over the five seeds: nominal 1.6 cm, 6.9 cm RMSE, NEES 2.2; push 1.7 cm, 8.6 cm RMSE, NEES 9.5.
* The push barely changes docking accuracy, but estimation RMSE and NEES are higher with the push **in every seed**.
* The filter needed its lock-out recovery only in seeds 3 and 4, which are also the two seeds with the highest push NEES. In the other three seeds the shove left little trace in these metrics.
* NEES here is the raw 3-degree-of-freedom value (ideal mean about 3, 95 % bound 7.8 for a single sample).
* Maximum-error figures at the very start of a run are the start-up transient before the first landmark update (for example 14 cm at 0.8 s in nominal seed 4), so they are not a disturbance effect.

**Case study: `push`, seed 4** (the matching nominal run is seed 4 as well).

||nominal|push|
|-|-|-|
|dock position / yaw error|1.4 cm / 1.4 deg|1.6 cm / 1.7 deg|
|estimation RMSE (xy)|4.9 cm|6.6 cm|
|NEES mean|3.56|11.44|
|x / y / yaw error inside the 3σ band|100 % / 100 % / 100 %|100 % / 93 % / 88 %|
|rejected updates|3|9|
|recoveries|0|1|
|replans|2|9|
|time to dock|42.2 s|49.8 s|

![push seed 4: estimate error with 3-sigma bounds and NEES](evidence/figures/push_seed4_estimation.png)

*Estimate minus truth with ±3σ bounds, NEES, and marker-visibility ticks for `push`, seed 4. The matching nominal figure is `evidence/figures/nominal_seed4_estimation.png`.*

Timeline, read from the log and the figures:

* **About 12.3 s (the shove).** The yaw error steps from about 1° to about 10° and NEES from about 3 to about 100. No marker is in view at that moment, and odometry cannot sense an external push, so the estimate does not move while the true pose jumps.
* **12.3 s to 18.3 s.** The yaw error stays outside the 3σ band, the y error leaves it from about 15 s and grows to about 19 cm (the worst xy error is 20.6 cm at 18.5 s), and NEES stays between 60 and 100. The covariance was already small from the earlier landmark updates, so it did not reflect the jump.
* **16 s to 18.3 s.** Markers reappear and several updates are rejected by the χ² gate: a small covariance plus a large jump makes correct measurements look like outliers. This is the case described in the caption of the figure in section 8.
* **About 18.3 s.** The lock-out recovery (section 5) fires, and the run reports one recovery. The covariance widens and the error collapses.
* **After 18.3 s.** The error returns inside the band, the final estimation error is 3.2 mm, and the robot docks 1.6 cm from the target.

This window of about 6 s is roughly 12 % of the run. At a NEES near 80 it contributes about 9.6 of the 11.4 mean, which matches the 88 % of samples that stay inside the bound. So the filter is not generally overconfident in this run; it is overconfident for the few seconds after an unmodelled shove, and the gate-recovery logic is what brings it back.

![push seed 4: trajectory with 95 % ellipses](evidence/figures/push_seed4_traj.png)

*True path, estimate and 95 % covariance ellipses (`plot_uncertainty.py`).*

**Ablation: odometry only vs landmark EKF (seed 3, same layout).**

||nominal (EKF)|`odom_only`|
|-|-|-|
|result|docked|failed|
|dock position error|3.4 cm|9.1 cm|
|dock yaw error|3.9 deg|5.2 deg|
|estimation RMSE|6.8 cm|17.6 cm|
|NEES|4.0|2.8|

Removing the landmark updates makes the estimation error about 2.6 times larger and the robot misses the dock. This single seed was on the mild side: over 10 seeds `odom_only` never docks and misses by about 38 cm (table above). The NEES of 2.8 shows that, with no corrections, the covariance grows in step with the real error. This is the observability result of section 3 seen in a run.

**Occlusion, seed 1.** Docked 0.8 cm / 0.6° from the target with an estimation RMSE of 5.4 cm and a NEES of 1.5, so a 10 s camera blackout did not hurt this run.

**Regenerating the evidence.**

```bash
python main.py --scenario nominal --seed 4 --record
python main.py --scenario push --seed 4 --record
python plot_uncertainty.py results/nominal_seed4_ekf     # traj_uncertainty.png, error_uncertainty.png
python plot_uncertainty.py results/push_seed4_ekf
python summarise.py                                      # table over results/*_ekf/metrics.json
```

`plot_uncertainty.py` also prints the NEES statistics, the share of samples inside the 3σ band per state, and the number of rejected updates for the run it plots. The runs are deterministic: repeating a seed reproduces its numbers exactly.

**Presentation demo video:** [https://youtu.be/OJ8svSIu7EU](https://youtu.be/OJ8svSIu7EU) is a single comparison video that brings the runs together. It is meant as the walkthrough; the separate clips below are the raw evidence behind it.

**Individual demo videos** (seed 4, nominal and push): 

https://github.com/user-attachments/assets/718115fb-1c5c-4ac2-a8f0-7dff18ef4431

https://github.com/user-attachments/assets/098123cb-5740-45c0-9ed8-8488d9c28547

In the robot-camera panel the dock target is boxed in green and the wall landmarks in orange. In the map panel the truth is green and the EKF estimate is orange with its 2σ ellipse, with a live readout of time, mode, position error, 2σ and yaw error. The videos run at twice real time. More clips are in `evidence/videos/` (nominal seeds 1 and 3, push seed 3, occlusion seed 1, odometry-only seed 3).

---

## 8. Beyond the brief

* **Data-driven noise models.** R as a function of apparent marker size, with an identified PnP-ambiguity threshold; Q identified from windowed odometry error.
* **Estimator comparison.** UKF vs EKF on paired seeds (`--estimators ekf ukf`):

|condition|scenario|estimator|n|success_rate|dock_pos_err|dock_yaw_err_deg|est_rmse_xy|nees_mean|track_e2_rmse|cross_track_rmse|effort|duration|
|-|-|-|-|-|-|-|-|-|-|-|-|-|
|default|nominal|ekf|10|100 %|2.04 ± 1.01 cm|2.16 ± 1.31|8.71 ± 4.04 cm|1.93 ± 1.04|2.73 ± 1.82 cm|7.89 ± 5.34 cm|0.38 ± 0.02|40.72 ± 3.53|
|default|nominal|ukf|10|100 %|1.97 ± 0.94 cm|2.07 ± 1.20|8.64 ± 4.19 cm|1.80 ± 0.80|2.70 ± 1.85 cm|7.88 ± 5.40 cm|0.38 ± 0.02|41.02 ± 3.54|
|default|stress|ekf|10|100 %|2.63 ± 1.25 cm|2.69 ± 1.50|9.85 ± 3.40 cm|12.54 ± 7.40|4.13 ± 1.58 cm|8.91 ± 4.37 cm|0.41 ± 0.03|40.20 ± 4.52|
|default|stress|ukf|10|90 %|11.13 ± 26.06 cm|4.61 ± 6.23|13.83 ± 12.72 cm|17.00 ± 10.20|4.30 ± 1.65 cm|10.77 ± 7.24 cm|0.40 ± 0.03|42.02 ± 5.62|

The UKF gives no measurable benefit here. At 20 Hz the motion model is nearly linear over one step, and the measurement
nonlinearity matters only very close to a marker, where the measurements are most accurate anyway. Both filters share the
same weakness, the unmodelled odometry *bias* in `stress`/`slip` (high NEES). 



![push trial](docs/figures/estimation_push.png)

*`push` scenario: a 25 N lateral shove at t = 12 s, invisible to odometry, makes the error jump by about 15 cm and 8°.
In this seed the shove happens before any marker is in range, so P is still large and the jump stays inside the 3σ envelope.
The first landmark updates pull the estimate back within a few seconds. When P is small, the same shove produces innovations
that fail the gate, which is why the gate-recovery logic exists (failure 1).*

---

## 9. Repository layout

```
main.py                     one-command demo (scenario / seed / video / viewer)
live_camera.py              live robot-camera demo (dock target green, wall landmarks orange)
plot_uncertainty.py         truth vs estimate with 3-sigma bands, trajectory with 95% ellipses, NEES
summarise.py                results table over results/*_ekf/metrics.json
assets/tb3/                 TurtleBot3 Waffle Pi MJCF + meshes (ROBOTIS, Apache-2.0) with G82 sensor additions
src/config.py               every parameter, scenario presets
src/hal/interface.py        hardware abstraction (no ground truth)
src/sim/                    world generator (room, obstacles, ArUco markers), MuJoCo robot, ground-truth oracle
src/perception/             camera model + extrinsics, ArUco + PnP measurements
src/estimation/             odometry, EKF, UKF
src/planning/               LiDAR occupancy grid, A*, path smoothing / reference
src/control/                LQR tracker, mission state machine + LiDAR safety layer
src/analysis/               controllability / observability / poles
src/evaluation/             logger + metrics (only user of ground truth), plots, video
scripts/                    calibration, noise identification, Monte-Carlo trials
tests/                      unit tests incl. the ground-truth-leak check
docs/                       model-analysis and calibration notes, Monte-Carlo and sweep results (docs/results), figures
evidence/                   videos, figures, per-run metrics, results table 
```

**Ground truth.** It is read only by `src/sim/ground_truth.py`, which `main.py`/`src/runner.py` call *after* the command for
the step is computed, plus the offline calibration scripts. `tests/test_pipeline.py::test_no_ground_truth_in_decision_code`
fails if any module in `estimation/`, `planning/`, `control/` or `perception/` references it or reads simulator state.

---

## 10. Acknowledgements

* TurtleBot3 Waffle Pi model and meshes: [ROBOTIS robotis_mujoco_menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie), Apache-2.0 (`assets/tb3/LICENSE`).
* 41014 weekly tutorial material (EKF, LQR); MuJoCo; OpenCV ArUco.
* References:

  * Thrun, Burgard and Fox, *Probabilistic Robotics* (velocity motion model, EKF localisation).
  * Kanayama et al. 1990 (tracking-error model).
  * Wan and van der Merwe 2000 (UKF).

