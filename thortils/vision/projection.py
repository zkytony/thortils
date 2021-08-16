# Projections

def inverse_perspective(u, v, d,
                        camera_intrinsics, camera_extrinsics):
    """
    Maps a pixel to 3D world
    Args:
        u, v, d: the horizontal, vertical coordinates of the pixel, and depth
        camera_intrinsics (np.array): The camera intrinsics matrix

               | fx  S   cx |
          K =  | 0   fy  cy |
               | 0   0   1  |

        camera_extrinsics (np.array): the camera extrinsics matrix (the affine)
    """
    raise NotImplementedError



def perspective(loc3d, camera_intrinsics, camera_extrinsics):
    """
    Maps a 3D world point to a pixel location

    Args:
        loc3d (tuple): x, y, z
    """
    raise NotImplementedError
