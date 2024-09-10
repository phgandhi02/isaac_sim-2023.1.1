# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})  # Option: "renderer": "PathTracing"

import numpy as np
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
from PIL import Image, ImageDraw

# Given the OpenCV camera matrix and distortion coefficients (Fisheye, Kannala-Brandt model),
# creates a camera and a sample scene, renders an image and saves it to
# camera_opencv_fisheye.png file. The asset is also saved to camera_opencv_fisheye.usd file.

# Currently only supports square images (there is an issue in the rendering pipeline).
# To produce non-square images, the region of the image that is not used should be cropped
width, height = 1920, 1200
camera_matrix = [[455.8, 0.0, 943.8], [0.0, 454.7, 602.3], [0.0, 0.0, 1.0]]
distortion_coefficients = [0.05, 0.01, -0.003, -0.0005]

# Camera sensor size and optical path parameters. These parameters are not the part of the
# OpenCV camera model, but they are nessesary to simulate the depth of field effect.
#
# To disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
pixel_size = 3  # in microns, 3 microns is common
f_stop = 1.8  # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
focus_distance = 0.6  # in meters, the distance from the camera to the object plane
diagonal_fov = 235  # in degrees, the diagonal field of view to be rendered


# Create a world, add a 1x1x1 meter cube, a ground plane, and a camera
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cube_1 = world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_1",
        name="cube_1",
        position=np.array([0, 0, 0.5]),
        scale=np.array([1.0, 1.0, 1.0]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)

cube_2 = world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_2",
        position=np.array([2, 0, 0.5]),
        scale=np.array([1.0, 1.0, 1.0]),
        size=1.0,
        color=np.array([0, 255, 0]),
    )
)

cube_3 = world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_3",
        name="cube_3",
        position=np.array([0, 4, 1]),
        scale=np.array([2.0, 2.0, 2.0]),
        size=1.0,
        color=np.array([0, 0, 255]),
    )
)

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 2.0]),  # 1 meter away from the side of the cube
    frequency=30,
    resolution=(width, height),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

# Setup the scene and render a frame
world.reset()
camera.initialize()

# Calculate the focal length and aperture size from the camera matrix
((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
horizontal_aperture = pixel_size * 1e-3 * width
vertical_aperture = pixel_size * 1e-3 * height
focal_length_x = fx * pixel_size * 1e-3
focal_length_y = fy * pixel_size * 1e-3
focal_length = (focal_length_x + focal_length_y) / 2  # in mm

# Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
camera.set_focal_length(focal_length / 10.0)
camera.set_focus_distance(focus_distance)
camera.set_lens_aperture(f_stop * 100.0)
camera.set_horizontal_aperture(horizontal_aperture / 10.0)
camera.set_vertical_aperture(vertical_aperture / 10.0)

camera.set_clipping_range(0.05, 1.0e5)

# Set the distortion coefficients
camera.set_projection_type("fisheyePolynomial")
camera.set_kannala_brandt_properties(width, height, cx, cy, diagonal_fov, distortion_coefficients)

# Get the rendered frame and save it to a file
for i in range(100):
    world.step(render=True)
camera.get_current_frame()
img = Image.fromarray(camera.get_rgba()[:, :, :3])

# Optional step, draw the 3D points to the image plane using the OpenCV fisheye model
def draw_points_opencv_fisheye(points3d):
    import cv2

    rvecs, tvecs = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
    points, jac = cv2.fisheye.projectPoints(
        np.expand_dims(points3d, 1), rvecs, tvecs, np.array(camera_matrix), np.array(distortion_coefficients)
    )
    draw = ImageDraw.Draw(img)
    for pt in points:
        x, y = pt[0]
        print("Drawing point at: ", x, y)
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill="yellow", outline="yellow")


# Draw a few 3D points at the image plane (camera is pointing down to the ground plane).
# OpenCV doen't support projecting points behind the camera, so we avoid that.
draw_points_opencv_fisheye(
    points3d=np.array(
        [
            [0.5, 0.5, 1.0],
            [-0.5, 0.5, 1.0],
            [0.5, -0.5, 1.0],
            [-0.5, -0.5, 1.0],
            [-3.0, -1.0, 0.0],
            [-3.0, 1.0, 0.0],
            [-0.5, -1.5, 1.0],
            [0.5, -1.5, 1.0],
        ]
    )
)


print("Saving the rendered image to: camera_opencv_fisheye.png")
img.save("camera_opencv_fisheye.png")

print("Saving the asset to camera_opencv_fisheye.usd")
world.scene.stage.Export("camera_opencv_fisheye.usd")

simulation_app.close()
