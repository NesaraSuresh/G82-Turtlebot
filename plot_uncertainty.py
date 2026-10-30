import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

run = sys.argv[1] if len(sys.argv) > 1 else r"results\push_seed4_ekf"
d = np.load(run + r"\log.npz")
t, xe, xt, P = d["t"], d["x_est"], d["x_true"], d["P"]

err = xe - xt
err[:, 2] = np.arctan2(np.sin(err[:, 2]), np.cos(err[:, 2]))
sig = np.sqrt(np.stack([P[:, i, i] for i in range(3)], axis=1))
nees = np.einsum("ni,nij,nj->n", err, np.linalg.inv(P), err)
rej = d["upd_t"][d["upd_acc"] == 0]

# ---- Figure 1: trajectory, truth vs estimate, with 95% ellipses ----
fig, ax = plt.subplots(figsize=(7, 7))
ax.plot(xt[:, 0], xt[:, 1], "k-", lw=2, label="true")
ax.plot(xe[:, 0], xe[:, 1], "b--", lw=1.5, label="estimate")
for i in range(0, len(t), 50):
    w, v = np.linalg.eigh(P[i, :2, :2])
    ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
    ax.add_patch(Ellipse(xe[i, :2], 2 * np.sqrt(5.991 * w[1]), 2 * np.sqrt(5.991 * w[0]),
                         angle=ang, fill=False, ec="tab:orange", lw=0.8,
                         label="95% ellipse" if i == 0 else None))
ax.plot(xt[0, 0], xt[0, 1], "go", label="start")
ax.plot(xt[-1, 0], xt[-1, 1], "rs", label="end")
ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.legend(); ax.grid(alpha=0.3)
ax.set_title("Trajectory: estimate vs true with uncertainty")
fig.savefig(run + r"\traj_uncertainty.png", dpi=150, bbox_inches="tight")

# ---- Figure 2: errors with 3-sigma bands, NEES, landmarks visible ----
fig, axs = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
scale = [100, 100, 180 / np.pi]
names = ["x error (cm)", "y error (cm)", "yaw error (deg)"]
for i in range(3):
    s = scale[i]
    axs[i].fill_between(t, -3 * sig[:, i] * s, 3 * sig[:, i] * s, color="tab:blue",
                        alpha=0.2, label="±3σ (filter)")
    axs[i].plot(t, err[:, i] * s, "k", lw=1, label="error (estimate - true)")
    axs[i].set_ylabel(names[i]); axs[i].grid(alpha=0.3)
axs[0].legend(loc="upper right")
axs[3].semilogy(t, nees, "k", lw=1)
axs[3].axhline(7.815, color="r", ls="--", label="95% bound (3 dof)")
axs[3].set_ylabel("NEES"); axs[3].legend(loc="upper right"); axs[3].grid(alpha=0.3)
nv = d["n_visible"].astype(float)
nv[nv < 0] = np.nan
for i in range(len(nv)):
    if np.isnan(nv[i]):
        nv[i] = nv[i - 1] if i > 0 else 0
axs[4].step(t, nv, where="post")
axs[4].set_ylabel("landmarks visible"); axs[4].set_xlabel("time (s)"); axs[4].grid(alpha=0.3)
for a in axs[:4]:
    for tt in rej:
        a.axvline(tt, color="r", alpha=0.25, lw=0.8)
axs[0].set_title("Estimate error with ±3σ uncertainty (red lines = rejected updates)")
fig.savefig(run + r"\error_uncertainty.png", dpi=150, bbox_inches="tight")

# ---- Numbers to check against metrics.json ----
k = int(np.argmax(np.hypot(err[:, 0], err[:, 1])))
print(f"mean NEES (3 dof, raw): {nees.mean():.2f}")
print(f"NEES within 95% bound:  {(nees < 7.815).mean() * 100:.0f}%")
print(f"rejected updates:       {len(rej)}")
print(f"max xy error:           {np.hypot(err[k, 0], err[k, 1]) * 100:.1f} cm at t={t[k]:.1f} s")
for i, n in enumerate(["x", "y", "yaw"]):
    print(f"{n} error inside 3σ:     {(np.abs(err[:, i]) < 3 * sig[:, i]).mean() * 100:.0f}%")
print("saved figures in", run)