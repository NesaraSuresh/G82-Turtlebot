import json, glob
from collections import defaultdict

rows = []
for f in sorted(glob.glob(r"results\*_ekf\metrics.json")):
    m = json.load(open(f))
    rows.append((m["scenario"], m["seed"], m["success"],
                 m["dock_pos_err"] * 100, m["dock_yaw_err_deg"],
                 m["est_rmse_xy"] * 100, m["nees_mean"], m["n_recoveries"]))

rows.sort(key=lambda r: (r[0], r[1]))
print("scenario   seed  success  dock_cm  yaw_deg  rmse_cm   nees  recov")
for r in rows:
    print(f"{r[0]:<10} {r[1]:<5} {str(r[2]):<8} {r[3]:7.1f}  {r[4]:7.2f}  {r[5]:7.1f}  {r[6]:5.1f}  {r[7]}")

print("\nMeans per scenario")
groups = defaultdict(list)
for r in rows:
    groups[r[0]].append(r)
for name, g in groups.items():
    n = len(g)
    ok = sum(1 for r in g if r[2])
    print(f"{name:<10} n={n}  success={ok}/{n}  dock_cm={sum(r[3] for r in g)/n:.1f}  "
          f"rmse_cm={sum(r[5] for r in g)/n:.1f}  nees={sum(r[6] for r in g)/n:.1f}")