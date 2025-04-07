import matplotlib.pyplot as plt


def visualize_voxel_grid_blocky(voxel_grid, perspective, title, debug=False):
    """
    Visualize a voxel grid in a blocky voxel-style using `matplotlib`.
    """
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the voxel grid
    ax.voxels(voxel_grid, facecolors='orange', edgecolor="k", linewidth=0.3)

    ax.set_xlim([0, voxel_grid.shape[0]])
    ax.set_ylim([0, voxel_grid.shape[1]])
    ax.set_zlim([0, voxel_grid.shape[2]])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.view_init(elev=perspective["elev"], azim=perspective["azim"])
    ax.set_title(title)

    plt.tight_layout()
    plt.show()


def visualize_multiple_views(voxel_grid, angles, title_prefix, debug=False):
    """
    Visualize voxel grids from multiple angles.
    """
    for i, angle in enumerate(angles):
        title = f"{title_prefix} - View {i + 1} (Elev: {angle['elev']}, Azim: {angle['azim']})"
        visualize_voxel_grid_blocky(voxel_grid, perspective=angle, title=title, debug=debug)