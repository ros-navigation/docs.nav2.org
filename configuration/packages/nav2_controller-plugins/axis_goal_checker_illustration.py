#!/usr/bin/env python3
"""
Illustration of the AxisGoalChecker algorithm.

This script generates a diagram showing how the axis goal checker determines
if a robot has reached its goal by projecting the distance along the path direction.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Circle, Wedge
import numpy as np


def draw_robot(ax, x, y, theta, color='blue', label='Robot'):
    """Draw a simplified robot representation."""
    # Robot body (circle)
    robot = Circle((x, y), 0.15, color=color, alpha=0.7, zorder=3)
    ax.add_patch(robot)

    # Direction indicator
    dx = 0.25 * np.cos(theta)
    dy = 0.25 * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.08,
             fc=color, ec=color, zorder=4)

    ax.plot(x, y, 'o', color=color, markersize=3, zorder=5)
    if label:
        ax.text(x, y - 0.4, label, ha='center', fontsize=9, fontweight='bold')


def draw_goal(ax, x, y, color='green', label='Goal'):
    """Draw a goal position."""
    goal = Circle((x, y), 0.06, color=color, alpha=0.5, zorder=2)
    ax.add_patch(goal)
    ax.plot(x, y, 'x', color='darkgreen', markersize=6, markeredgewidth=1.5, zorder=3)
    if label:
        ax.text(x, y + 0.4, label, ha='center', fontsize=9, fontweight='bold')
        ax.text(x, y + 0.65, '(path end)', ha='center', fontsize=7, style='italic', color='gray')


def draw_path_point(ax, x, y, color='orange', label='Path Point'):
    """Draw the before-goal path point (second to last)."""
    point = Circle((x, y), 0.08, color=color, alpha=0.7, zorder=2)
    ax.add_patch(point)
    ax.plot(x, y, 'o', color='darkorange', markersize=6, zorder=3)
    if label:
        ax.text(x, y - 0.5, label, ha='center', fontsize=9, fontweight='bold')
        ax.text(x, y - 0.75, '(penultimate)', ha='center', fontsize=7, style='italic', color='gray')


def create_illustration():
    """Create the main illustration."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Scenario 1: Within tolerance (goal reached)
    ax1.set_xlim(-0.5, 5.5)
    ax1.set_ylim(0, 4.5)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Scenario 1: Goal Reached',
                  fontsize=12, fontweight='bold')

    # Define positions for scenario 1
    # Full path with multiple points (evenly spaced, larger scale)
    path_1 = [(0.5, 0.8), (1.5, 1.4), (2.5, 2.0), (3.5, 2.6)]
    before_goal_1 = path_1[-2]  # Second to last
    goal_1 = path_1[-1]  # Last (goal_pose)
    robot_1 = (3.15, 2.85)  # Closer to goal but still visible projection
    robot_theta_1 = np.pi / 6

    # Draw full path with all points
    path_x = [p[0] for p in path_1]
    path_y = [p[1] for p in path_1]
    ax1.plot(path_x, path_y, 'k--', linewidth=1.5, alpha=0.3, label='Full Path')

    # Draw earlier path points
    for i in range(len(path_1) - 2):
        ax1.plot(path_1[i][0], path_1[i][1], 'o', color='gray', markersize=4, alpha=0.5, zorder=2)

    # Highlight the last segment
    ax1.plot([before_goal_1[0], goal_1[0]], [before_goal_1[1], goal_1[1]],
             'k-', linewidth=3, alpha=0.7, label='Last Path Segment')

    # Calculate angles for scenario 1
    end_of_path_yaw_1 = np.arctan2(
        goal_1[1] - before_goal_1[1],
        goal_1[0] - before_goal_1[0]
    )
    robot_to_goal_yaw_1 = np.arctan2(
        goal_1[1] - robot_1[1],
        goal_1[0] - robot_1[0]
    )

    # Calculate and draw projection
    distance_to_goal = np.hypot(goal_1[0] - robot_1[0], goal_1[1] - robot_1[1])
    projection_angle_1 = robot_to_goal_yaw_1 - end_of_path_yaw_1
    projected_distance_1 = distance_to_goal * np.cos(projection_angle_1)
    ortho_projected_distance_1 = distance_to_goal * np.sin(projection_angle_1)

    # Projection point on path axis
    proj_point_1 = (
        goal_1[0] - projected_distance_1 * np.cos(end_of_path_yaw_1),
        goal_1[1] - projected_distance_1 * np.sin(end_of_path_yaw_1)
    )

    # Draw projections
    ax1.plot([robot_1[0], proj_point_1[0]], [robot_1[1], proj_point_1[1]],
             'orange', linewidth=2.5, alpha=0.7, linestyle='--',
             label='Cross-Track Distance')
    ax1.plot([proj_point_1[0], goal_1[0]], [proj_point_1[1], goal_1[1]],
             'lime', linewidth=4, alpha=0.7,
             label='Along-Path Distance')

    # Draw 2D tolerance zones showing both is_overshoot_valid modes
    along_path_tolerance = 0.30
    cross_track_tolerance = 1.50

    from matplotlib.transforms import Affine2D
    # is_overshoot_valid=false: symmetric zone
    tolerance_rect_sym = mpatches.Rectangle(
        (-along_path_tolerance, -cross_track_tolerance),
        2 * along_path_tolerance, 2 * cross_track_tolerance,
        edgecolor='blue', facecolor='lightblue',
        alpha=0.15, linewidth=2, linestyle='-',
        transform=Affine2D().rotate(end_of_path_yaw_1).translate(goal_1[0], goal_1[1]) + ax1.transData,
        label='is_overshoot_valid=false'
    )
    ax1.add_patch(tolerance_rect_sym)

    # is_overshoot_valid=true: infinite forward, limited backward
    infinite_length = 5.0
    tolerance_rect_fwd = mpatches.Rectangle(
        (-along_path_tolerance, -cross_track_tolerance),
        infinite_length + along_path_tolerance, 2 * cross_track_tolerance,
        edgecolor='green', facecolor='lightgreen',
        alpha=0.2, linewidth=2, linestyle='-',
        transform=Affine2D().rotate(end_of_path_yaw_1).translate(goal_1[0], goal_1[1]) + ax1.transData,
        label='is_overshoot_valid=true'
    )
    ax1.add_patch(tolerance_rect_fwd)

    draw_goal(ax1, goal_1[0], goal_1[1], label='')
    draw_robot(ax1, robot_1[0], robot_1[1], robot_theta_1)

    ax1.legend(loc='upper left', fontsize=8)
    ax1.set_xlabel('X (meters)', fontsize=10)
    ax1.set_ylabel('Y (meters)', fontsize=10)

    # Add result text
    result_text_1 = '✓ GOAL REACHED\nBoth tolerances satisfied'
    ax1.text(2.5, 0.4, result_text_1, ha='center', fontsize=9,
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8),
             fontweight='bold')

    # Scenario 2: Beyond tolerance (goal not reached)
    ax2.set_xlim(-0.5, 5.5)
    ax2.set_ylim(0, 4.5)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Scenario 2: Goal Not Reached',
                  fontsize=12, fontweight='bold')

    # Define positions for scenario 2
    # Full path with multiple points (evenly spaced, larger scale)
    path_2 = [(0.5, 0.8), (1.5, 1.4), (2.5, 2.0), (3.5, 2.6)]
    before_goal_2 = path_2[-2]  # Second to last
    goal_2 = path_2[-1]  # Last (goal_pose)
    robot_2 = (2.0, 3.2)  # Further away
    robot_theta_2 = np.pi / 4

    # Draw full path with all points
    path_x_2 = [p[0] for p in path_2]
    path_y_2 = [p[1] for p in path_2]
    ax2.plot(path_x_2, path_y_2, 'k--', linewidth=1.5, alpha=0.3, label='Full Path')

    # Draw earlier path points
    for i in range(len(path_2) - 2):
        ax2.plot(path_2[i][0], path_2[i][1], 'o', color='gray', markersize=4, alpha=0.5, zorder=2)

    # Highlight the last segment
    ax2.plot([before_goal_2[0], goal_2[0]], [before_goal_2[1], goal_2[1]],
             'k-', linewidth=3, alpha=0.7, label='Last Path Segment')

    # Calculate angles for scenario 2
    end_of_path_yaw_2 = np.arctan2(
        goal_2[1] - before_goal_2[1],
        goal_2[0] - before_goal_2[0]
    )
    robot_to_goal_yaw_2 = np.arctan2(
        goal_2[1] - robot_2[1],
        goal_2[0] - robot_2[0]
    )

    # Calculate and draw projection
    distance_to_goal_2 = np.hypot(goal_2[0] - robot_2[0], goal_2[1] - robot_2[1])
    projection_angle_2 = robot_to_goal_yaw_2 - end_of_path_yaw_2
    projected_distance_2 = distance_to_goal_2 * np.cos(projection_angle_2)
    ortho_projected_distance_2 = distance_to_goal_2 * np.sin(projection_angle_2)

    # Projection point on path axis
    proj_point_2 = (
        goal_2[0] - projected_distance_2 * np.cos(end_of_path_yaw_2),
        goal_2[1] - projected_distance_2 * np.sin(end_of_path_yaw_2)
    )

    # Draw projections
    ax2.plot([robot_2[0], proj_point_2[0]], [robot_2[1], proj_point_2[1]],
             'orange', linewidth=2.5, alpha=0.7, linestyle='--',
             label='Cross-Track Distance')
    ax2.plot([proj_point_2[0], goal_2[0]], [proj_point_2[1], goal_2[1]],
             'red', linewidth=4, alpha=0.7,
             label='Along-Path Distance')

    # Draw 2D tolerance zones showing both is_overshoot_valid modes
    along_path_tolerance_2 = 0.30
    cross_track_tolerance_2 = 1.50

    from matplotlib.transforms import Affine2D
    # is_overshoot_valid=false: symmetric zone
    tolerance_rect2_sym = mpatches.Rectangle(
        (-along_path_tolerance_2, -cross_track_tolerance_2),
        2 * along_path_tolerance_2, 2 * cross_track_tolerance_2,
        edgecolor='blue', facecolor='lightblue',
        alpha=0.15, linewidth=2, linestyle='-',
        transform=Affine2D().rotate(end_of_path_yaw_2).translate(goal_2[0], goal_2[1]) + ax2.transData,
        label='is_overshoot_valid=false'
    )
    ax2.add_patch(tolerance_rect2_sym)

    # is_overshoot_valid=true: infinite forward, limited backward
    infinite_length_2 = 5.0
    tolerance_rect2_fwd = mpatches.Rectangle(
        (-along_path_tolerance_2, -cross_track_tolerance_2),
        infinite_length_2 + along_path_tolerance_2, 2 * cross_track_tolerance_2,
        edgecolor='green', facecolor='lightgreen',
        alpha=0.2, linewidth=2, linestyle='-',
        transform=Affine2D().rotate(end_of_path_yaw_2).translate(goal_2[0], goal_2[1]) + ax2.transData,
        label='is_overshoot_valid=true'
    )
    ax2.add_patch(tolerance_rect2_fwd)

    draw_goal(ax2, goal_2[0], goal_2[1], label='')
    draw_robot(ax2, robot_2[0], robot_2[1], robot_theta_2)

    ax2.legend(loc='upper left', fontsize=8)
    ax2.set_xlabel('X (meters)', fontsize=10)
    ax2.set_ylabel('Y (meters)', fontsize=10)

    # Add result text
    result_text_2 = '✗ GOAL NOT REACHED\nAlong-path tolerance exceeded'
    ax2.text(2.5, 0.4, result_text_2, ha='center', fontsize=9,
             bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8),
             fontweight='bold')

    # Main title
    fig.suptitle('AxisGoalChecker: Path-Projection Based Goal Detection',
                 fontsize=14, fontweight='bold', y=0.98)

    # Add explanation
    explanation = (
        "The AxisGoalChecker projects the robot's position onto the path direction. Goal is reached when along-path and cross-track distances are within tolerance.\n"
        "is_overshoot_valid=true: allows robot ANY distance past goal (green zone infinite forward, checks: along_path_distance < tolerance). "
        "is_overshoot_valid=false: requires robot within tolerance on both sides (blue zone symmetric, checks: fabs(along_path_distance) < tolerance)."
    )
    fig.text(0.5, 0.02, explanation, ha='center', fontsize=10,
             style='italic', wrap=True)

    plt.tight_layout(rect=[0, 0.05, 1, 0.96])

    return fig


if __name__ == '__main__':
    fig = create_illustration()

    # Save to Nav2 docs folder where RST looks for it
    output_path = '/opt/auto_ws/src/auto-sandbox/src/vendor/docs.nav2.org/images/axis_goal_checker.png'
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved to: {output_path}")

    plt.show()
