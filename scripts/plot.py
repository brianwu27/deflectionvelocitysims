import csv, sys, os
from collections import defaultdict
import matplotlib.pyplot as plt

# ------------------- Inputs & knobs -------------------
in_path = sys.argv[1] if len(sys.argv) > 1 else "out/deflection_log.csv"

# Commanded-angle binning and plateau detection
BIN_DEG       = 1.0
EPS_DEG       = 0.25
SLOPE_THRESH  = 20.0    # deg/s; how flat is "flat"
MIN_PLATEAU_S = 0.05    # s minimum plateau to count

# ------------------- Load CSV -------------------
t, vz, cmd, tgt = [], [], [], []
with open(in_path, newline="") as f:
    r = csv.DictReader(f)
    for row in r:
        t.append(float(row["time_s"]))
        vz.append(float(row["vel_z_mps"]))
        cmd.append(float(row["deflection_deg"]))
        tgt.append(float(row["target_deg"]))

# ------------------- Plot (target + commanded) -------------------
fig, ax1 = plt.subplots()
ax1.set_xlabel("time [s]")
ax1.set_ylabel("vertical velocity [m/s]")
ax1.plot(t, vz, label="vz")

ax2 = ax1.twinx()
ax2.set_ylabel("deflection [deg]")
ax2.plot(t, tgt, linestyle=":", label="target")
ax2.plot(t, cmd, linestyle="--", label="deflection cmd")
ax2.scatter(t, cmd, s=6, alpha=0.35, label="cmd samples")

lns = ax1.get_lines() + ax2.get_lines()
labs = [l.get_label() for l in lns]
ax1.legend(lns, labs, loc="best")

plt.title("Velocity vs. Deflection (target & commanded)")
plt.tight_layout()
plt.show()

# ------------------- Helpers -------------------
def bin_center(x, step=BIN_DEG):
    return round(x / step) * step

def finite_diff(xs, ts, i):
    if i == 0: return 0.0
    dt = max(1e-9, ts[i] - ts[i-1])
    return (xs[i] - xs[i-1]) / dt

# ------------------- Plateau detection (commanded-only) -------------------
plateaus = []  
cur = None

for i in range(len(t)):
    cbin   = bin_center(cmd[i])
    inband = abs(cmd[i] - cbin) <= EPS_DEG
    slope  = abs(finite_diff(cmd, t, i))
    flat   = slope <= SLOPE_THRESH

    if inband and flat:
        if cur is None:
            cur = {"bin": cbin, "t_start": t[i], "indices": [i]}
        else:
            if abs(cur["bin"] - cbin) <= 1e-12:
                cur["indices"].append(i)
            else:
                cur["t_end"] = t[i]
                plateaus.append(cur)
                cur = {"bin": cbin, "t_start": t[i], "indices": [i]}
    else:
        if cur is not None:
            cur["t_end"] = t[i]
            plateaus.append(cur)
            cur = None

if cur is not None:
    cur["t_end"] = t[-1]
    plateaus.append(cur)

plateaus = [p for p in plateaus if (p["t_end"] - p["t_start"]) >= MIN_PLATEAU_S]

# ------------------- Build commanded-only coverage + sorted CSV -------------------
os.makedirs("out", exist_ok=True)

raw_rows = []
per_bin_vels = defaultdict(list)

for p in plateaus:
    b = p["bin"]
    for i in p["indices"]:
        raw_rows.append({
            "time_s": t[i],
            "cmd_bin_deg": b,
            "vel_z_mps": vz[i]
        })
        per_bin_vels[b].append(vz[i])

# ✅ SORT rows by command angle then time
raw_rows.sort(key=lambda r: (r["cmd_bin_deg"], r["time_s"]))

raw_path = "out/velocities_by_cmd_plateau_raw.csv"
with open(raw_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["time_s", "cmd_bin_deg", "vel_z_mps"])
    w.writeheader()
    w.writerows(raw_rows)

# ------------------- Compact stats per angle -------------------
stats_rows = []
for b in sorted(per_bin_vels.keys()):
    vs = per_bin_vels[b]
    stats_rows.append({
        "cmd_bin_deg": b,
        "count_samples": len(vs),
        "vel_min_mps": min(vs),
        "vel_mean_mps": sum(vs)/len(vs),
        "vel_max_mps": max(vs)
    })

stats_path = "out/coverage_by_cmd_plateau_stats.csv"
with open(stats_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(stats_rows[0].keys()))
    w.writeheader()
    w.writerows(stats_rows)

# Print to terminal
print("\n=== Commanded-only coverage (plateau-based) ===")
for r in stats_rows:
    print(f"cmd≈{r['cmd_bin_deg']:>5.1f}°  "
          f"n={r['count_samples']:>4d}  "
          f"v[min, mean, max]=[{r['vel_min_mps']:.2f}, {r['vel_mean_mps']:.2f}, {r['vel_max_mps']:.2f}] m/s")

print(f"\nWrote sorted per-sample velocities: {raw_path}")
print(f"Wrote stats table:                 {stats_path}")
