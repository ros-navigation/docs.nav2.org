#!/usr/bin/env python3
"""
Visualization of AdaptiveToleranceGoalChecker — simulates the full decision
logic with four robot trajectories, one per acceptance path:

  1. Fine zone instant accept
  2. Finish line crossing (entry-position based)
  3. Stopped stagnation
  4. Distance stagnation (moving but not improving)

Each subplot shows cycle-by-cycle state, tolerance zones, the dynamic
finish line (perpendicular to approach direction), and counter values.
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ── Plugin parameters (match test defaults) ─────────────────────────────
FINE_XY_TOL = 0.10
COARSE_XY_TOL = 0.25
TRANS_STOPPED_VEL = 0.10
ROT_STOPPED_VEL = 0.10
REQUIRED_STAGNATION_CYCLES = 5  # smaller than default 15 for visual clarity


# ── Pure-Python replica of the checker ──────────────────────────────────
class AdaptiveToleranceGoalChecker:
    def __init__(self):
        self.fine_sq = FINE_XY_TOL ** 2
        self.coarse_sq = COARSE_XY_TOL ** 2
        self.reset()

    def reset(self):
        self.in_zone = False
        self.stopped_count = 0
        self.distance_count = 0
        self.best_dist_sq = float("inf")
        self.approach_dx = 0.0
        self.approach_dy = 0.0
        self.entry_x = None
        self.entry_y = None
        self.log = []  # per-cycle debug info

    def check(self, rx, ry, gx, gy, lin_vel, ang_vel):
        dx = rx - gx
        dy = ry - gy
        dist_sq = dx * dx + dy * dy

        info = dict(
            rx=rx, ry=ry, dist=math.sqrt(dist_sq),
            in_zone=self.in_zone,
            stopped_count=self.stopped_count,
            distance_count=self.distance_count,
            finish_dot=None, crossed=False,
            reason=None, accepted=False,
        )

        # Fine zone
        if dist_sq <= self.fine_sq:
            info["reason"] = "fine"
            info["accepted"] = True
            self.log.append(info)
            return True, info

        # Coarse zone
        if dist_sq <= self.coarse_sq:
            if not self.in_zone:
                self.in_zone = True
                self.stopped_count = 0
                self.distance_count = 0
                self.best_dist_sq = dist_sq
                self.approach_dx = -dx
                self.approach_dy = -dy
                self.entry_x = rx
                self.entry_y = ry
                info["in_zone"] = True
                info["reason"] = "zone_entry"
                self.log.append(info)
                return False, info

            finish_dot = dx * self.approach_dx + dy * self.approach_dy
            crossed = finish_dot >= 0.0
            info["finish_dot"] = finish_dot
            info["crossed"] = crossed

            robot_stopped = (
                math.hypot(lin_vel, 0) <= TRANS_STOPPED_VEL
                and abs(ang_vel) <= ROT_STOPPED_VEL
            )
            improved = dist_sq < self.best_dist_sq
            if improved:
                self.best_dist_sq = dist_sq

            if robot_stopped:
                self.stopped_count += 1
            else:
                self.stopped_count = 0

            if improved:
                self.distance_count = 0
            else:
                self.distance_count += 1

            info["stopped_count"] = self.stopped_count
            info["distance_count"] = self.distance_count

            if crossed:
                info["reason"] = "finish_line"
                info["accepted"] = True
            elif self.stopped_count >= REQUIRED_STAGNATION_CYCLES:
                info["reason"] = "stopped_stagnation"
                info["accepted"] = True
            elif self.distance_count >= REQUIRED_STAGNATION_CYCLES:
                info["reason"] = "distance_stagnation"
                info["accepted"] = True
            else:
                info["reason"] = "waiting"

            self.log.append(info)
            return info["accepted"], info

        # Outside
        self.in_zone = False
        self.stopped_count = 0
        self.distance_count = 0
        info["reason"] = "outside"
        self.log.append(info)
        return False, info


# ── Scenario definitions ────────────────────────────────────────────────
GOAL = (0.0, 0.0)


def scenario_fine_zone():
    """Robot approaches from left, enters fine zone → instant accept."""
    gc = AdaptiveToleranceGoalChecker()
    pts = [
        (-0.35, 0.05, 0.20, 0.0),   # outside
        (-0.22, 0.03, 0.15, 0.0),   # coarse entry
        (-0.14, 0.02, 0.12, 0.0),   # approaching in coarse
        (-0.07, 0.01, 0.10, 0.0),   # still coarse
        (-0.03, 0.00, 0.05, 0.0),   # fine zone → accept
    ]
    results = []
    for rx, ry, lv, av in pts:
        accepted, info = gc.check(rx, ry, *GOAL, lv, av)
        results.append(info)
        if accepted:
            break
    return gc, results


def scenario_finish_line():
    """Robot passes by the goal offset to the side — never enters fine zone."""
    gc = AdaptiveToleranceGoalChecker()
    # Robot moves left-to-right at y ≈ -0.13, offset below goal.
    # Closest approach ~0.13m (stays in coarse band, never hits fine zone).
    # Entry at (-0.19, -0.15): approach = (0.19, 0.15)
    # Finish line crossed when robot passes the goal laterally.
    pts = [
        (-0.32, -0.16, 0.20, 0.0),   # outside
        (-0.19, -0.15, 0.18, 0.0),   # coarse entry (offset below goal), dist=0.242
        (-0.08, -0.14, 0.15, 0.0),   # approaching, dist=0.161, dot < 0
        ( 0.05, -0.13, 0.12, 0.0),   # closest point, dist=0.139, dot ≈ -0.01
        ( 0.16, -0.12, 0.12, 0.0),   # passed goal! dist=0.200, dot > 0 → accept
    ]
    results = []
    for rx, ry, lv, av in pts:
        accepted, info = gc.check(rx, ry, *GOAL, lv, av)
        results.append(info)
        if accepted:
            break
    return gc, results


def scenario_stopped_stagnation():
    """Robot enters coarse zone then stops completely → accept after N cycles."""
    gc = AdaptiveToleranceGoalChecker()
    # Enter, then stay at same position with zero velocity
    stop_x, stop_y = -0.16, 0.08
    pts = [(-0.35, 0.15, 0.20, 0.0)]  # outside
    pts.append((stop_x, stop_y, 0.12, 0.0))  # coarse entry (moving)
    pts.append((stop_x, stop_y, 0.0, 0.0))  # stopped, cycle 1
    for i in range(REQUIRED_STAGNATION_CYCLES):
        pts.append((stop_x, stop_y, 0.0, 0.0))
    results = []
    for rx, ry, lv, av in pts:
        accepted, info = gc.check(rx, ry, *GOAL, lv, av)
        results.append(info)
        if accepted:
            break
    return gc, results


def scenario_distance_stagnation():
    """Robot keeps moving (velocity > threshold) but orbits without improving distance."""
    gc = AdaptiveToleranceGoalChecker()
    # Enter at one point, then orbit at constant radius in coarse zone
    radius = 0.18
    entry_angle = math.pi * 0.8
    pts = [(-0.35, 0.10, 0.20, 0.0)]  # outside
    # Entry point
    ex = radius * math.cos(entry_angle)
    ey = radius * math.sin(entry_angle)
    pts.append((ex, ey, 0.15, 0.0))  # coarse entry
    # Orbit: constant radius, so distance never improves
    n_orbit = REQUIRED_STAGNATION_CYCLES + 1
    for i in range(n_orbit):
        angle = entry_angle + (i + 1) * 0.25
        ox = radius * math.cos(angle)
        oy = radius * math.sin(angle)
        pts.append((ox, oy, 0.15, 0.0))  # moving, not improving
    results = []
    for rx, ry, lv, av in pts:
        accepted, info = gc.check(rx, ry, *GOAL, lv, av)
        results.append(info)
        if accepted:
            break
    return gc, results


# ── Plotting ────────────────────────────────────────────────────────────
REASON_COLORS = {
    "outside": "#9E9E9E",
    "zone_entry": "#2196F3",
    "waiting": "#FF9800",
    "fine": "#4CAF50",
    "finish_line": "#E91E63",
    "stopped_stagnation": "#9C27B0",
    "distance_stagnation": "#795548",
}

TITLES = [
    "Path 1: Fine Zone — Instant Accept",
    "Path 2: Finish Line Crossing",
    "Path 3: Stopped Stagnation",
    "Path 4: Distance Stagnation (orbiting)",
]


def draw_zones(ax, goal_yaw=0.0):
    """Draw coarse and fine tolerance circles + goal marker with orientation."""
    coarse = patches.Circle(
        GOAL, COARSE_XY_TOL, fill=True,
        facecolor="#FFE0B2", edgecolor="#FF9800", linewidth=1.8,
        linestyle="--", zorder=1, alpha=0.6,
    )
    fine = patches.Circle(
        GOAL, FINE_XY_TOL, fill=True,
        facecolor="#C8E6C9", edgecolor="#4CAF50", linewidth=1.8,
        zorder=2, alpha=0.6,
    )
    ax.add_patch(coarse)
    ax.add_patch(fine)
    ax.plot(*GOAL, "ko", markersize=7, zorder=10)
    # Goal orientation arrow (thick black)
    arr_len = 0.06
    ax.annotate(
        "", xy=(GOAL[0] + arr_len * math.cos(goal_yaw),
                GOAL[1] + arr_len * math.sin(goal_yaw)),
        xytext=GOAL,
        arrowprops=dict(arrowstyle="-|>", color="black", lw=2.5,
                        mutation_scale=12),
        zorder=10,
    )
    ax.annotate(
        "Goal\n(yaw→)", GOAL, textcoords="offset points",
        xytext=(6, -18), fontsize=7, fontweight="bold", zorder=10,
    )


def draw_finish_line(ax, gc):
    """Draw the dynamic finish line perpendicular to approach direction."""
    if gc.approach_dx == 0.0 and gc.approach_dy == 0.0:
        return
    # Normalize approach direction
    mag = math.hypot(gc.approach_dx, gc.approach_dy)
    if mag < 1e-9:
        return
    adx = gc.approach_dx / mag
    ady = gc.approach_dy / mag
    # Perpendicular
    px, py = -ady, adx
    line_len = COARSE_XY_TOL + 0.03
    ax.plot(
        [GOAL[0] - px * line_len, GOAL[0] + px * line_len],
        [GOAL[1] - py * line_len, GOAL[1] + py * line_len],
        color="#E91E63", linewidth=2, linestyle="-", alpha=0.6, zorder=4,
    )
    # Small arrow showing approach direction
    arr_scale = 0.08
    ax.annotate(
        "", xy=(GOAL[0] + adx * arr_scale, GOAL[1] + ady * arr_scale),
        xytext=GOAL,
        arrowprops=dict(arrowstyle="->", color="#E91E63", lw=1.5),
        zorder=5,
    )
    # Label the two half-planes
    ax.text(
        GOAL[0] - adx * 0.19, GOAL[1] - ady * 0.19,
        "dot<0", fontsize=7, color="#1565C0", ha="center",
        style="italic", alpha=0.7, zorder=5,
    )
    ax.text(
        GOAL[0] + adx * 0.19, GOAL[1] + ady * 0.19,
        "dot≥0", fontsize=7, color="#C62828", ha="center",
        style="italic", alpha=0.7, zorder=5,
    )


def draw_entry_marker(ax, gc):
    """Mark the zone entry point with a diamond and draw approach direction to goal."""
    if gc.entry_x is not None:
        ax.plot(
            gc.entry_x, gc.entry_y, "D",
            color="#2196F3", markersize=9, markeredgecolor="white",
            markeredgewidth=1.2, zorder=12,
        )
        ax.annotate(
            "entry", (gc.entry_x, gc.entry_y),
            textcoords="offset points", xytext=(-8, 10),
            fontsize=7, color="#2196F3", fontweight="bold",
        )
        # Draw dotted green line from entry to goal showing approach direction
        # (the finish line is perpendicular to THIS, not to the robot heading)
        ax.plot(
            [gc.entry_x, GOAL[0]], [gc.entry_y, GOAL[1]],
            color="#2E7D32", linewidth=2.5, linestyle=":", alpha=0.7, zorder=5,
        )
        # Small arrowhead at goal end
        ax.annotate(
            "", xy=GOAL, xytext=(gc.entry_x, gc.entry_y),
            arrowprops=dict(
                arrowstyle="-|>", color="#2E7D32", lw=0.01,
                mutation_scale=12, alpha=0.7,
            ),
            zorder=5,
        )
        # Label it
        mid_x = (gc.entry_x + GOAL[0]) / 2
        mid_y = (gc.entry_y + GOAL[1]) / 2
        ax.annotate(
            "approach dir\n(entry→goal)", (mid_x, mid_y),
            textcoords="offset points", xytext=(-45, -16),
            fontsize=6.5, color="#2E7D32", fontweight="bold",
            alpha=0.85, zorder=5,
        )


def draw_scenario(ax, gc, results, title, goal_yaw=0.0):
    draw_zones(ax, goal_yaw=goal_yaw)
    draw_finish_line(ax, gc)
    draw_entry_marker(ax, gc)

    xs = [r["rx"] for r in results]
    ys = [r["ry"] for r in results]

    # Trajectory line
    ax.plot(xs, ys, "-", color="#607D8B", linewidth=1.2, alpha=0.5, zorder=6)

    # Compute headings from consecutive positions (direction of travel)
    headings = []
    for i in range(len(results)):
        if i < len(results) - 1:
            hdx = results[i + 1]["rx"] - results[i]["rx"]
            hdy = results[i + 1]["ry"] - results[i]["ry"]
            headings.append(math.atan2(hdy, hdx))
        else:
            headings.append(headings[-1] if headings else 0.0)

    # Per-cycle dots + orientation arrows
    arrow_len = 0.04
    for i, r in enumerate(results):
        color = REASON_COLORS.get(r["reason"], "#000")
        marker = "*" if r["accepted"] else "o"
        size = 14 if r["accepted"] else 7
        edge = "white" if r["accepted"] else color
        ax.plot(
            r["rx"], r["ry"], marker=marker, color=color,
            markersize=size, markeredgecolor=edge,
            markeredgewidth=0.8, zorder=11,
        )
        # Orientation arrow (larger on entry point)
        yaw = headings[i]
        is_entry = (r["reason"] == "zone_entry")
        a_len = 0.06 if is_entry else arrow_len
        a_lw = 2.2 if is_entry else 1.3
        a_scale = 11 if is_entry else 8
        ax.annotate(
            "", xy=(r["rx"] + a_len * math.cos(yaw),
                     r["ry"] + a_len * math.sin(yaw)),
            xytext=(r["rx"], r["ry"]),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=a_lw,
                            mutation_scale=a_scale),
            zorder=11,
        )
        # Cycle number
        ax.annotate(
            str(i), (r["rx"], r["ry"]),
            textcoords="offset points", xytext=(5, 5),
            fontsize=6, color="#333", zorder=12,
        )

    # Acceptance annotation — always in top-right corner with arrow to accept point
    last = results[-1]
    if last["accepted"]:
        label_map = {
            "fine": "ACCEPT\n(fine zone)",
            "finish_line": "ACCEPT\n(finish line crossed)",
            "stopped_stagnation": f"ACCEPT\n(stopped {REQUIRED_STAGNATION_CYCLES} cycles)",
            "distance_stagnation": f"ACCEPT\n(no improvement\n{REQUIRED_STAGNATION_CYCLES} cycles)",
        }
        label = label_map.get(last["reason"], "ACCEPT")
        ax.annotate(
            label, (last["rx"], last["ry"]),
            xytext=(0.95, 0.95), textcoords="axes fraction",
            fontsize=8, fontweight="bold", ha="right", va="top",
            bbox=dict(
                boxstyle="round,pad=0.3", facecolor="white",
                edgecolor=REASON_COLORS.get(last["reason"], "gray"), alpha=0.95,
            ),
            arrowprops=dict(arrowstyle="->", color="#333"),
            zorder=13,
        )

    # Counters annotation (for stagnation scenarios)
    counter_lines = []
    for r in results:
        if r["reason"] in ("waiting", "stopped_stagnation", "distance_stagnation"):
            counter_lines.append(
                f"  cyc: stop={r['stopped_count']} dist={r['distance_count']}"
            )
    if counter_lines:
        # Show last few counter states
        shown = counter_lines[-min(4, len(counter_lines)):]
        text = "Counters (last cycles):\n" + "\n".join(shown)
        ax.text(
            0.02, 0.02, text, fontsize=6.5, family="monospace",
            transform=ax.transAxes, verticalalignment="bottom",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow",
                      edgecolor="gray", alpha=0.9),
            zorder=14,
        )

    ax.set_xlim(-0.42, 0.42)
    ax.set_ylim(-0.35, 0.35)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("X (m)", fontsize=8)
    ax.set_ylabel("Y (m)", fontsize=8)
    ax.set_title(title, fontsize=10, fontweight="bold")


# ── Main ────────────────────────────────────────────────────────────────
def main():
    scenarios = [
        scenario_fine_zone,
        scenario_finish_line,
        scenario_stopped_stagnation,
        scenario_distance_stagnation,
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(
        "AdaptiveToleranceGoalChecker — Four Acceptance Paths\n"
        f"(fine={FINE_XY_TOL}m, coarse={COARSE_XY_TOL}m, "
        f"stagnation_cycles={REQUIRED_STAGNATION_CYCLES})",
        fontsize=13, fontweight="bold", y=0.98,
    )

    # Each scenario gets a different goal yaw to show it's independent of finish line
    goal_yaws = [
        0.0,                    # Path 1: goal faces right
        math.pi / 2,           # Path 2: goal faces up (perpendicular to approach!)
        -math.pi * 0.75,       # Path 3: goal faces down-left
        math.pi * 0.3,         # Path 4: goal faces up-right
    ]

    for ax, scenario_fn, title, goal_yaw in zip(axes.flat, scenarios, TITLES, goal_yaws):
        gc, results = scenario_fn()
        draw_scenario(ax, gc, results, title, goal_yaw=goal_yaw)

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        patches.Patch(facecolor="#C8E6C9", edgecolor="#4CAF50", label=f"Fine zone (≤{FINE_XY_TOL}m)"),
        patches.Patch(facecolor="#FFE0B2", edgecolor="#FF9800", label=f"Coarse zone ({FINE_XY_TOL}–{COARSE_XY_TOL}m)"),
        Line2D([0], [0], color="#E91E63", linewidth=2, label="Finish line (⊥ approach dir)"),
        Line2D([0], [0], color="#2E7D32", linewidth=2, linestyle=":", label="Approach dir (entry→goal)"),
        Line2D([0], [0], marker="D", color="#2196F3", linestyle="None", markersize=7, label="Zone entry point"),
        Line2D([0], [0], marker="o", color="black", linestyle="None", markersize=7,
               markeredgewidth=2.5, label="Goal pose (arrow = yaw, independent)"),
        Line2D([0], [0], marker="*", color="#4CAF50", linestyle="None", markersize=12, label="Accept (fine)"),
        Line2D([0], [0], marker="*", color="#E91E63", linestyle="None", markersize=12, label="Accept (finish line)"),
        Line2D([0], [0], marker="*", color="#9C27B0", linestyle="None", markersize=12, label="Accept (stopped stagnation)"),
        Line2D([0], [0], marker="*", color="#795548", linestyle="None", markersize=12, label="Accept (distance stagnation)"),
    ]
    fig.legend(
        handles=legend_elements, loc="lower center",
        ncol=4, fontsize=8.5, framealpha=0.9,
        bbox_to_anchor=(0.5, 0.005),
    )

    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    out_path = "/opt/auto_ws/src/auto-sandbox/src/vendor/docs.nav2.org/images/adaptive_tolerance_goal_checker.png"
    plt.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"Saved → {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
