import pandas as pd
import matplotlib.pyplot as plt

# Read the log produced by agv_sim.exe
df = pd.read_csv("../run_log.csv")

# Assign a color per state so transitions are visually obvious
state_colors = {
    "IDLE": "gray",
    "PATH_PLANNING": "orange",
    "MOVING": "green",
    "OBSTACLE_DETECTED": "red",
    "REROUTING": "purple",
    "DOCKED": "blue",
    "ERROR": "black",
}

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# --- Top plot: position over time, colored by state ---
for state, color in state_colors.items():
    mask = df["state"] == state
    ax1.scatter(df.loc[mask, "time_s"], df.loc[mask, "position_cm"],
                s=4, color=color, label=state)

ax1.set_ylabel("Position (cm)")
ax1.set_title("AGV Position Over Time, Colored by State")
ax1.legend(markerscale=3, loc="upper left", fontsize=8)
ax1.grid(True, alpha=0.3)

# --- Bottom plot: distance to nearest obstacle/dock over time ---
ax2.plot(df["time_s"], df["distance_cm"], color="darkred", linewidth=1)
ax2.axhline(y=15, color="gray", linestyle="--", linewidth=1, label="obstacle threshold (15cm)")
ax2.set_ylabel("Distance ahead (cm)")
ax2.set_xlabel("Time (s)")
ax2.set_title("Distance to Nearest Obstacle/Dock")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("run_plot.png", dpi=150)
print("Saved chart to visualizer/run_plot.png")
plt.show()