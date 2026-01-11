import numpy as np
import time
import pybullet as p
from scipy.interpolate import CubicSpline



import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection


from scipy.spatial import HalfspaceIntersection, ConvexHull
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from matplotlib.animation import FuncAnimation



from collections import Counter

import os
import shutil
from matplotlib.animation import FuncAnimation


'''
This module contains helper functions for visualization and geometric computations.
'''
def plot_rrt_mpc_2d(path_xyz,
                    spline_xyz,
                    maze_walls,
                    mpc_obstacles_info,
                    drone_traj_xyz,
                    start_pos,
                    goal_pos,
                    save_path=None):
    """
    2D (x-y) plot with z shown via:
      - line color (altitude colormap)
      - markers over/under obstacles footprint:
          ▲  = drone is OVER an obstacle footprint (z > obstacle_top)
          ▼  = drone is UNDER an obstacle footprint (z < obstacle_bottom)

    Obstacles drawn as:
      - maze_walls: rectangles in x-y using (ox,oy,hx,hy)
      - mpc_obstacles_info: circles in x-y using center + radius
    """

    path_xyz = np.asarray(path_xyz)
    spline_xyz = np.asarray(spline_xyz)
    drone_traj_xyz = np.asarray(drone_traj_xyz)

    fig, ax = plt.subplots(figsize=(9, 7))

    def add_colored_xy_line(ax, xy, z, linewidth=3.0, linestyle='-', label=None):
        """Draw x-y polyline colored by z using a LineCollection."""
        xy = np.asarray(xy)
        z = np.asarray(z)

        pts = xy.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)

        lc = LineCollection(segs, linewidths=linewidth, linestyles=linestyle)
        lc.set_array(z[:-1])  # color per segment by altitude
        ax.add_collection(lc)

        # Legend 
        if label is not None:
            ax.plot([], [], linestyle=linestyle, linewidth=linewidth, label=label)

        return lc

    def point_in_wall_xy(x, y, wall):
        ox, oy, oz, hx, hy, hz = wall
        return (ox - hx <= x <= ox + hx) and (oy - hy <= y <= oy + hy)

    def wall_z_bounds(wall):
        ox, oy, oz, hx, hy, hz = wall
        return (oz - hz, oz + hz)

    def point_in_circle_xy(x, y, cx, cy, r):
        return (x - cx) ** 2 + (y - cy) ** 2 <= r ** 2

    #  draw maze walls (rectangles)
    for wall in maze_walls:
        ox, oy, oz, hx, hy, hz = wall
        rect = Rectangle((ox - hx, oy - hy), 2 * hx, 2 * hy,
                         fill=True, alpha=0.15, linewidth=1.5)
        ax.add_patch(rect)

    # draw MPC obstacles (circles)
    for o in mpc_obstacles_info:
        c = o.get('pos', [0, 0, 0])
        r = float(o.get('r', 0.2))
        cx, cy = float(c[0]), float(c[1])
        circ = Circle((cx, cy), r, fill=True, alpha=0.20, linewidth=1.5)
        ax.add_patch(circ)

    # plot RRT discrete path (thin dashed)
    ax.plot(path_xyz[:, 0], path_xyz[:, 1], '--', linewidth=1.5, label='RRT path (discrete)')

 # plot spline reference (SINGLE COLOR)

    ax.plot(
        spline_xyz[:, 0], spline_xyz[:, 1],
        linewidth=3.0,
        linestyle='--',   
        color='blue',     
        label='RRT spline (reference)'
    )

    # plot drone actual trajectory (ALTITUDE COLORED)
    lc_real = add_colored_xy_line(
        ax,
        xy=drone_traj_xyz[:, 0:2],
        z=drone_traj_xyz[:, 2],
        linewidth=4.0,
        linestyle='-',
        label='Drone trajectory (MPC, colored by z)'
    )


    # mark "over" / "under" obstacle footprints
    over_x, over_y = [], []
    under_x, under_y = [], []

    for (x, y, z) in drone_traj_xyz:
        # 1) check walls
        for wall in maze_walls:
            if point_in_wall_xy(x, y, wall):
                zmin, zmax = wall_z_bounds(wall)
                if z > zmax:
                    over_x.append(x); over_y.append(y)
                elif z < zmin:
                    under_x.append(x); under_y.append(y)

        #check circular MPC obstacles

        for o in mpc_obstacles_info:
            c = o.get('pos', [0, 0, 0])
            r = float(o.get('r', 0.2))
            cx, cy, cz = float(c[0]), float(c[1]), float(c[2])
            if point_in_circle_xy(x, y, cx, cy, r):
                zmin, zmax = (cz - r), (cz + r)
                if z > zmax:
                    over_x.append(x); over_y.append(y)
                elif z < zmin:
                    under_x.append(x); under_y.append(y)

    if len(over_x) > 0:
        ax.scatter(over_x, over_y, marker='^', s=40, label='OVER obstacle footprint (▲)')
    if len(under_x) > 0:
        ax.scatter(under_x, under_y, marker='v', s=40, label='UNDER obstacle footprint (▼)')

    # ---- start/goal ----
    ax.scatter([start_pos[0]], [start_pos[1]], s=80, marker='o', label='Start')
    ax.scatter([goal_pos[0]],  [goal_pos[1]],  s=80, marker='*', label='Goal')

    ax.set_title("RRT reference vs MPC trajectory (2D x-y) with altitude encoding")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)

    cbar = fig.colorbar(lc_real, ax=ax)
    cbar.set_label("Altitude z [m] (line color)")


    ax.legend(
        loc='upper center',
        bbox_to_anchor=(0.5, -0.12),
        ncol=3,
        fontsize=9,
        frameon=True
    )

    
    plt.subplots_adjust(bottom=0.20)


    
    all_x = np.concatenate([path_xyz[:, 0], spline_xyz[:, 0], drone_traj_xyz[:, 0]])
    all_y = np.concatenate([path_xyz[:, 1], spline_xyz[:, 1], drone_traj_xyz[:, 1]])
    margin = 1.0
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    if save_path is not None:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')

    plt.show()

def halfspaces_xy_from_Ab_at_z(A3, b, z0):
    """
    Convert 3D halfspaces A3 x <= b into 2D halfspaces (x,y) at fixed z=z0:
      ax*x + ay*y + az*z0 <= b  ->  ax*x + ay*y + (az*z0 - b) <= 0
    SciPy expects: [ax, ay, c] meaning ax*x + ay*y + c <= 0
    """
    hs = []
    for i in range(A3.shape[0]):
        ax, ay, az = A3[i]
        c = az * z0 - b[i]
        hs.append([ax, ay, c])
    return np.array(hs, dtype=float)


def add_bbox_halfspaces(xmin, xmax, ymin, ymax):
    """Bounding box to keep intersection bounded."""
    return np.array([
        [ 1.0,  0.0, -xmax],  # x <= xmax
        [-1.0,  0.0,  xmin],  # x >= xmin
        [ 0.0,  1.0, -ymax],  # y <= ymax
        [ 0.0, -1.0,  ymin],  # y >= ymin
    ], dtype=float)


def polygon_from_halfspaces(halfspaces, interior_point_xy):
    """
    halfspaces: (K,3) rows [a,b,c] meaning a*x + b*y + c <= 0
    interior_point_xy must be feasible.
    Returns polygon vertices in CCW order or None.
    """
    try:
        hs_int = HalfspaceIntersection(halfspaces, interior_point_xy)
        verts = hs_int.intersections
        if verts.shape[0] < 3:
            return None
        hull = ConvexHull(verts)
        return verts[hull.vertices]
    except Exception:
        return None


def _line_segment_in_bbox(a, b, c, xmin, xmax, ymin, ymax):
    """
    Find intersection segment of line a*x + b*y + c = 0 with the bbox.
    Returns two points (p1, p2) or None if it doesn't cross bbox.
    """
    pts = []

    
    if abs(b) > 1e-12:
        y = (-c - a * xmin) / b
        if ymin - 1e-9 <= y <= ymax + 1e-9:
            pts.append((xmin, y))
        y = (-c - a * xmax) / b
        if ymin - 1e-9 <= y <= ymax + 1e-9:
            pts.append((xmax, y))

    
    if abs(a) > 1e-12:
        x = (-c - b * ymin) / a
        if xmin - 1e-9 <= x <= xmax + 1e-9:
            pts.append((x, ymin))
        x = (-c - b * ymax) / a
        if xmin - 1e-9 <= x <= xmax + 1e-9:
            pts.append((x, ymax))

    # Remove near-duplicates
    uniq = []
    for p in pts:
        if all((abs(p[0]-q[0]) > 1e-6 or abs(p[1]-q[1]) > 1e-6) for q in uniq):
            uniq.append(p)

    if len(uniq) < 2:
        return None

    # pick farthest two points (robust)
    best = None
    best_d = -1.0
    for i in range(len(uniq)):
        for j in range(i+1, len(uniq)):
            dx = uniq[i][0] - uniq[j][0]
            dy = uniq[i][1] - uniq[j][1]
            d = dx*dx + dy*dy
            if d > best_d:
                best_d = d
                best = (uniq[i], uniq[j])
    return best


def plot_convex_snapshot_xy(drone_traj, convex_log, obs_centers, obs_radii, r_drone,
                            bbox=(-2, 12, -2, 16), snap_index=None):
    """
    1) Clear snapshot plot:
       - drone path
       - obstacles
       - ONE convex region polygon (filled)
       - half-space boundary lines + arrows (feasible side)
    """
    xmin, xmax, ymin, ymax = bbox

    if len(convex_log) == 0:
        print("convex_log is empty, nothing to plot.")
        return

    # pick snapshot index: default = middle
    if snap_index is None:
        snap_index = len(convex_log) // 2
    snap_index = int(np.clip(snap_index, 0, len(convex_log)-1))

    entry = convex_log[snap_index]
    A3 = entry["A"]
    b = entry["b"]
    z0 = entry["z"]
    pxy = entry["pos"][:2]

    hs = halfspaces_xy_from_Ab_at_z(A3, b, z0)
    hs_bbox = add_bbox_halfspaces(xmin, xmax, ymin, ymax)
    hs_all = np.vstack([hs, hs_bbox])

    poly = polygon_from_halfspaces(hs_all, pxy)

    fig, ax = plt.subplots(figsize=(9, 7))

    # drone path
    ax.plot(drone_traj[:,0], drone_traj[:,1], linewidth=2.5, label="Drone XY path")
    ax.scatter([pxy[0]], [pxy[1]], s=60, label=f"Snapshot (k={snap_index})")

    # obstacles (inflated for visualization)
    margin = 0.05
    for c_obs, r_obs in zip(obs_centers, obs_radii):
        cx, cy = float(c_obs[0]), float(c_obs[1])
        R = float(r_obs) + float(r_drone) + margin
        ax.add_patch(Circle((cx, cy), R, alpha=0.12))
        ax.add_patch(Circle((cx, cy), float(r_obs), fill=False, linewidth=1.0, alpha=0.6))

    # polygon fill
    if poly is not None:
        ax.add_patch(Polygon(poly, closed=True, alpha=0.18, label="Convex feasible set (slice)"))
        ax.plot(poly[:,0], poly[:,1], linewidth=2.0, alpha=0.5)

    # draw half-space boundary lines (one per obstacle constraint)
    for i in range(hs.shape[0]):
        a, bb, c = hs[i]

        seg = _line_segment_in_bbox(a, bb, c, xmin, xmax, ymin, ymax)
        if seg is None:
            continue
        (x1, y1), (x2, y2) = seg
        ax.plot([x1, x2], [y1, y2], linestyle="--", linewidth=1.5, alpha=0.7)

        # Arrow showing feasible side
        n = np.array([a, bb], dtype=float)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        feasible_dir = -n  

        
        xm, ym = 0.5*(x1+x2), 0.5*(y1+y2)

   
        val = a*pxy[0] + bb*pxy[1] + c
        
        if val > 1e-6:
            feasible_dir = -feasible_dir

        ax.arrow(xm, ym, 0.7*feasible_dir[0], 0.7*feasible_dir[1],
                 head_width=0.12, head_length=0.18, length_includes_head=True, alpha=0.6)

    ax.set_title("ONE snapshot: convex feasible set + half-space boundaries")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    plt.show()



def animate_convex_tunnel_xy(drone_traj, convex_log, obs_centers, obs_radii, r_drone,
                             bbox=(-2, 12, -2, 16), stride=8, interval_ms=80,
                             save_path=None, dpi=150):
    """
    Saves an animation of the convex safe bubble evolving along the trajectory.
    Handles empty local constraints by defaulting to the bounding box.
    """
    xmin, xmax, ymin, ymax = bbox

    # Robust check for data
    if not convex_log or len(convex_log) == 0:
        print("convex_log is empty, nothing to animate.")
        return None

    # Subsample indices for performance
    idxs = np.arange(0, len(convex_log), max(1, int(stride)))
    if len(idxs) == 0:
        idxs = np.array([0]) 

    fig, ax = plt.subplots(figsize=(9, 7))

    # Static elements: Full drone path and obstacle footprints
    ax.plot(drone_traj[:,0], drone_traj[:,1], linewidth=2.0, alpha=0.5, label="Drone XY path")
    
    margin = 0.05
    for c_obs, r_obs in zip(obs_centers, obs_radii):
        cx, cy = float(c_obs[0]), float(c_obs[1])
        R = float(r_obs) + float(r_drone) + margin
        ax.add_patch(Circle((cx, cy), R, alpha=0.10))
        ax.add_patch(Circle((cx, cy), float(r_obs), fill=False, linewidth=1.0, alpha=0.6))

    # Dynamic elements: Polygon for safe region and scatter for current position
    poly_patch = Polygon(np.zeros((3,2)), closed=True, alpha=0.18, label="Local Safe Region")
    ax.add_patch(poly_patch)
    point_scatter = ax.scatter([], [], s=50, color='red')
    title = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontweight='bold')

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")

    def init():
        poly_patch.set_xy(np.zeros((3,2)))
        point_scatter.set_offsets(np.empty((0,2)))
        title.set_text("")
        return poly_patch, point_scatter, title

    def update(frame_i):
        idx = idxs[frame_i]
        entry = convex_log[idx]
        A3 = entry["A"]
        b = entry["b"]
        z0 = entry["z"]
        pxy = entry["pos"][:2]

        # Convert 3D halfspaces to 2D slice at height z0
        hs = halfspaces_xy_from_Ab_at_z(A3, b, z0)
        
        
        if hs.size > 0:
            # Combine local obstacles with the map bounding box
            hs_all = np.vstack([hs, add_bbox_halfspaces(xmin, xmax, ymin, ymax)])
        else:
            # No nearby obstacles: Safe region is just the bounding box
            hs_all = add_bbox_halfspaces(xmin, xmax, ymin, ymax)

        # Generate the polygon vertices
        poly = polygon_from_halfspaces(hs_all, pxy)
        if poly is None:
            poly_patch.set_xy(np.zeros((3,2)))
        else:
            poly_patch.set_xy(poly)

        point_scatter.set_offsets(np.array([pxy]))
        title.set_text(f"Convex set evolution | snapshot {idx}/{len(convex_log)}")

        return poly_patch, point_scatter, title

    ani = FuncAnimation(
        fig, update,
        frames=range(len(idxs)), 
        init_func=init,
        interval=interval_ms,
        blit=False,
        repeat=True
    )

    if save_path is not None:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        ext = os.path.splitext(save_path)[1].lower()
        fps = max(1, int(1000 / interval_ms))
        
        try:
            if ext == ".mp4" and shutil.which("ffmpeg"):
                ani.save(save_path, writer="ffmpeg", fps=fps, dpi=dpi)
                print(f"Saved MP4: {save_path}")
            else:
                ani.save(save_path, writer="pillow", fps=fps, dpi=dpi)
                print(f"Saved GIF: {save_path}")
        except Exception as e:
            print(f"Animation save failed: {e}")

    plt.show()
    return ani

def plot_convex_frames_grid(drone_traj, convex_log, obs_centers, obs_radii, r_drone,
                            bbox=(-2, 12, -2, 16), rows=2, cols=4):
    """
    Displays a static grid of snapshots showing the safe region at different time steps.
    """
    xmin, xmax, ymin, ymax = bbox

    if not convex_log or len(convex_log) == 0:
        print("convex_log is empty, nothing to plot.")
        return

    n = rows * cols
    idxs = np.linspace(0, len(convex_log)-1, n).astype(int)

    fig, axs = plt.subplots(rows, cols, figsize=(4.5*cols, 4.0*rows), sharex=True, sharey=True)
    axs = np.array(axs).reshape(-1)

    margin = 0.05

    for ax, idx in zip(axs, idxs):
        entry = convex_log[idx]
        A3 = entry["A"]
        b = entry["b"]
        z0 = entry["z"]
        pxy = entry["pos"][:2]

        # Background: Path and current position
        ax.plot(drone_traj[:,0], drone_traj[:,1], linewidth=1.5, alpha=0.4)
        ax.scatter([pxy[0]], [pxy[1]], s=35, color='red')

        # Static Obstacles
        for c_obs, r_obs in zip(obs_centers, obs_radii):
            cx, cy = float(c_obs[0]), float(c_obs[1])
            R = float(r_obs) + float(r_drone) + margin
            ax.add_patch(Circle((cx, cy), R, alpha=0.08))

       
        hs = halfspaces_xy_from_Ab_at_z(A3, b, z0)
        if hs.size > 0:
            hs_all = np.vstack([hs, add_bbox_halfspaces(xmin, xmax, ymin, ymax)])
        else:
            hs_all = add_bbox_halfspaces(xmin, xmax, ymin, ymax)

        # Polygon generation
        poly = polygon_from_halfspaces(hs_all, pxy)
        if poly is not None:
            ax.add_patch(Polygon(poly, closed=True, alpha=0.14, color='blue'))
            ax.plot(poly[:,0], poly[:,1], linewidth=1.0, alpha=0.5, color='blue')

        ax.set_title(f"k={idx}", fontsize=10)
        ax.grid(True, alpha=0.2)
        ax.set_aspect("equal", adjustable="box")

    for ax in axs:
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)

    fig.suptitle("Convex feasible set evolution (frames grid)", fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def plot_mpc_solver_stats(ctrl, save_path=None, show=True):
    """
    Plot and print solver performance metrics from your MPC controller.

    Expects ctrl to have (lists):
      - ctrl.solve_times  : list of solve times in seconds
      - ctrl.solve_status : list of cvxpy statuses (strings)
      - ctrl.solve_iters  : list of iteration counts (or NaN)

    Usage (after simulation):
      plot_mpc_solver_stats(ctrl, save_path="results/solver_stats.png")
    """


    if not hasattr(ctrl, "solve_times") or len(getattr(ctrl, "solve_times", [])) == 0:
        print("[plot_mpc_solver_stats] No solver logs found on ctrl (solve_times empty/missing).")
        print("Make sure you logged them inside MPC.computeControlFromState().")
        return None

    solve_times = np.asarray(ctrl.solve_times, dtype=float)  # seconds
    solve_ms = 1000.0 * solve_times

    statuses = list(getattr(ctrl, "solve_status", ["unknown"] * len(solve_ms)))
    iters = np.asarray(getattr(ctrl, "solve_iters", [np.nan] * len(solve_ms)), dtype=float)

    # --- Summary prints ---
    print("\n=== MPC Solver Stats ===")
    print(f"Steps logged:      {len(solve_ms)}")
    print(f"Mean solve time:   {np.mean(solve_ms):.2f} ms")
    print(f"Median solve time: {np.median(solve_ms):.2f} ms")
    print(f"95th percentile:   {np.percentile(solve_ms, 95):.2f} ms")
    print(f"Max solve time:    {np.max(solve_ms):.2f} ms")

    finite_iters = iters[np.isfinite(iters)]
    if finite_iters.size > 0:
        print(f"Mean iterations:   {np.mean(finite_iters):.1f}")
        print(f"Max iterations:    {np.max(finite_iters):.0f}")

    print("Status counts:", dict(Counter(statuses)))

    # Make status numeric for plotting
    uniq = list(dict.fromkeys(statuses))  # preserve order of first appearance
    status_to_id = {s: i for i, s in enumerate(uniq)}
    status_ids = np.array([status_to_id[s] for s in statuses], dtype=int)

    # --- Plot ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(solve_ms)
    axs[0].set_ylabel("Solve time [ms]")
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(iters)
    axs[1].set_ylabel("Clarabel iterations")
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(status_ids)
    axs[2].set_ylabel("Solver status")
    axs[2].set_xlabel("MPC step")
    axs[2].set_yticks(list(status_to_id.values()))
    axs[2].set_yticklabels(list(status_to_id.keys()))
    axs[2].grid(True, alpha=0.3)

    fig.suptitle("MPC Solver Performance (Clarabel)")
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[plot_mpc_solver_stats] Saved figure to: {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    
    return {
        "solve_ms": solve_ms,
        "iters": iters,
        "statuses": statuses,
        "status_to_id": status_to_id
    }