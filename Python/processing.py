import numpy as np
import scipy.ndimage
import trimesh


def load_stl_file(stl_filename):
    """
    Load an STL file using the Trimesh library.
    """
    return trimesh.load_mesh(stl_filename)


def normalize_mesh(mesh, debug=False):
    """
    Normalize the mesh to fit exactly into a unit cube (1x1x1) centered at the origin.
    """
    mesh.apply_translation(-mesh.center_mass)
    bounding_box_size = mesh.bounding_box.extents
    largest_dimension = np.max(bounding_box_size)
    scale_factor = 1.0 / largest_dimension
    mesh.apply_scale(scale_factor)

    if debug:
        print("Bounding box before scaling:", bounding_box_size)
        print("Uniform scaling factor applied:", scale_factor)

    return mesh


def voxelize_mesh(mesh, grid_size, debug=False):
    """
    Voxelizes the given mesh into a proportional grid of booleans.
    """
    voxel_pitch = 1.0 / grid_size
    voxels = mesh.voxelized(voxel_pitch)

    if debug:
        print("Voxel pitch:", voxel_pitch)
        print("Voxel grid shape:", voxels.matrix.shape)

    return voxels.matrix


def resize_to_target_size(voxel_grid, target_shape=(64, 64, 64), debug=False):
    """
    Resize or pad the voxel grid to an exact shape, preserving content proportions.
    """
    current_shape = np.array(voxel_grid.shape)
    scale_factors = target_shape / current_shape
    min_scale = np.min(scale_factors)

    proportional_resized = scipy.ndimage.zoom(
        voxel_grid.astype(float),
        zoom=[min_scale] * 3,
        order=1
    ) > 0.5

    resized_grid = np.zeros(target_shape, dtype=bool)
    for axis in range(3):
        diff = target_shape[axis] - proportional_resized.shape[axis]
        if diff > 0:
            pad_before = diff // 2
            pad_after = diff - pad_before
            proportional_resized = np.pad(
                proportional_resized,
                [(pad_before, pad_after) if i == axis else (0, 0) for i in range(3)],
                mode='constant',
                constant_values=False
            )
        elif diff < 0:
            start = -diff // 2
            proportional_resized = proportional_resized[
                tuple(
                    slice(start, start + target_shape[axis]) if i == axis else slice(None)
                    for i in range(3)
                )
            ]

    resized_grid = proportional_resized
    if debug:
        print("Final resized grid shape:", resized_grid.shape)

    return resized_grid


def stl_to_fixed_size_voxels(stl_filename, target_shape=(64, 64, 64), debug=False):
    """
    Converts an STL file to a fixed-size voxel array, preserving proportions.
    """
    mesh = load_stl_file(stl_filename)
    if debug:
        print("Loaded mesh.")

    normalize_mesh(mesh, debug=debug)
    voxel_grid = voxelize_mesh(mesh, grid_size=max(target_shape), debug=debug)
    fixed_voxel_grid = resize_to_target_size(voxel_grid, target_shape=target_shape, debug=debug)

    return fixed_voxel_grid


def ensure_binary_voxel_grid(voxel_grid):
    """
    Ensures the voxel grid has only 0 and 1 values.
    Positive values become 1, and non-positive values become 0.

    Args:
    voxel_grid (np.array): Input voxel grid.

    Returns:
    np.array: Binary voxel grid (0 and 1).
    """
    return (np.asarray(voxel_grid) > 0).astype(int)


def downsample_with_threshold(high_res_voxel_grid, target_shape=(8, 8, 8), threshold=0.1, debug=False):
    """
    Downsample a high-resolution voxel grid to a lower resolution with a specified threshold.
    """
    scale_factors = tuple(
        high_res_voxel_grid.shape[i] // target_shape[i] for i in range(3)
    )

    downsampled_grid = np.zeros(target_shape, dtype=bool)

    for x in range(target_shape[0]):
        for y in range(target_shape[1]):
            for z in range(target_shape[2]):
                x_start, x_end = x * scale_factors[0], (x + 1) * scale_factors[0]
                y_start, y_end = y * scale_factors[1], (y + 1) * scale_factors[1]
                z_start, z_end = z * scale_factors[2], (z + 1) * scale_factors[2]

                block = high_res_voxel_grid[
                        x_start:x_end, y_start:y_end, z_start:z_end
                        ]
                if block.mean() >= threshold:  # At least threshold% of voxels are active
                    downsampled_grid[x, y, z] = True

    if debug:
        print(f"Downsampled grid shape: {downsampled_grid.shape}")
        # print(f"Downsampled grid: \n{downsampled_grid}")  # Add this line to print the array

        return downsampled_grid
    return downsampled_grid
