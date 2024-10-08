# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


"""Helper class for obtaining groundtruth data from OmniKit.

Support provided for RGB, Depth, Bounding Box (2D Tight, 2D Loose, 3D),
segmentation (instance and semantic), and camera parameters.

    Typical usage example:

    kit = OmniKitHelper()   # Start omniverse kit
    sd_helper = SyntheticDataHelper()
    gt = sd_helper.get_groundtruth(['rgb', 'depth', 'boundingBox2DTight'], viewport)

"""

import asyncio
import builtins
import math
import time
import typing
import warnings

import carb
import numpy as np
import omni


class SyntheticDataHelper:
    def __init__(self):
        self.app = omni.kit.app.get_app_interface()
        ext_manager = self.app.get_extension_manager()
        ext_manager.set_extension_enabled("omni.syntheticdata", True)

        from omni.syntheticdata import helpers, sensors

        self.sd = omni.syntheticdata._syntheticdata

        self.sd_interface = omni.syntheticdata._syntheticdata.acquire_syntheticdata_interface()
        self.carb_settings = carb.settings.acquire_settings_interface()
        self.sensor_helper_lib = sensors
        self.generic_helper_lib = helpers

        self.sensor_helpers = {
            "rgb": sensors.get_rgb,
            "depth": sensors.get_distance_to_image_plane,
            "depthLinear": sensors.get_distance_to_image_plane,
            "instanceSegmentation": sensors.get_instance_segmentation,
            "semanticSegmentation": sensors.get_semantic_segmentation,
            "boundingBox2DTight": sensors.get_bounding_box_2d_tight,
            "boundingBox2DLoose": sensors.get_bounding_box_2d_loose,
            "boundingBox3D": sensors.get_bounding_box_3d,
            "camera": self.get_camera_params,
            "pose": self.get_pose,
            "occlusion": sensors.get_occlusion,
        }

        self.sensor_types = {
            "rgb": omni.syntheticdata._syntheticdata.SensorType.Rgb,
            "depth": omni.syntheticdata._syntheticdata.SensorType.DistanceToImagePlane,
            "depthLinear": omni.syntheticdata._syntheticdata.SensorType.DistanceToImagePlane,
            "instanceSegmentation": omni.syntheticdata._syntheticdata.SensorType.InstanceSegmentation,
            "semanticSegmentation": omni.syntheticdata._syntheticdata.SensorType.SemanticSegmentation,
            "boundingBox2DTight": omni.syntheticdata._syntheticdata.SensorType.BoundingBox2DTight,
            "boundingBox2DLoose": omni.syntheticdata._syntheticdata.SensorType.BoundingBox2DLoose,
            "boundingBox3D": omni.syntheticdata._syntheticdata.SensorType.BoundingBox3D,
            "occlusion": omni.syntheticdata._syntheticdata.SensorType.Occlusion,
        }

        self.sensor_state = {s: False for s in list(self.sensor_helpers.keys())}

    def get_camera_params(self, viewport):
        """Get active camera intrinsic and extrinsic parameters.

        Returns:
            A dict of the active camera's parameters.

            pose (numpy.ndarray): camera position in world coordinates,
            fov (float): horizontal field of view in radians
            focal_length (float)
            horizontal_aperture (float)
            view_projection_matrix (numpy.ndarray(dtype=float64, shape=(4, 4)))
            resolution (dict): resolution as a dict with 'width' and 'height'.
            clipping_range (tuple(float, float)): Near and Far clipping values.
        """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(viewport.get_active_camera())
        prim_tf = omni.usd.get_world_transform_matrix(prim)
        view_params = self.generic_helper_lib.get_view_params(viewport)
        fov = 2 * math.atan(view_params["horizontal_aperture"] / (2 * view_params["focal_length"]))
        view_proj_mat = self.generic_helper_lib.get_view_proj_mat(view_params)

        return {
            "pose": np.array(prim_tf),
            "fov": fov,
            "focal_length": view_params["focal_length"],
            "horizontal_aperture": view_params["horizontal_aperture"],
            "view_projection_matrix": view_proj_mat,
            "resolution": {"width": view_params["width"], "height": view_params["height"]},
            "clipping_range": view_params["clipping_range"],
        }

    def get_pose(self):
        """Get pose of all objects with a semantic label."""
        stage = omni.usd.get_context().get_stage()
        mappings = self.generic_helper_lib.get_instance_mappings()
        pose = []
        for m in mappings:
            prim_path = m[1]
            prim = stage.GetPrimAtPath(prim_path)
            prim_tf = omni.usd.get_world_transform_matrix(prim)
            pose.append((str(prim_path), m[2], str(m[3]), np.array(prim_tf)))
        return pose

    def initialize(self, sensor_names, viewport_api):
        """Initialize sensors in the list provided.


        Args:
            viewport_api (Any): Viewport from which to retrieve/create sensor.
            sensor_types (list of omni.syntheticdata._syntheticdata.SensorType): List of sensor types to initialize.
        """
        for sensor_name in sensor_names:
            if sensor_name != "camera" and sensor_name != "pose":
                self.sensor_helper_lib.enable_sensors(viewport_api, [self.sensor_types[sensor_name]])
                if builtins.ISAAC_LAUNCHED_FROM_JUPYTER:
                    data = []
                    while data == []:
                        self.app.update()
                        data = self.sensor_helpers[sensor_name](viewport_api)
                else:
                    future = asyncio.ensure_future(self.sensor_helper_lib.next_sensor_data_async(viewport_api))
                    while not future.done():
                        self.app.update()
        self.app.update()

    async def initialize_async(self, sensor_names, viewport_api):
        """Initialize sensors in the list provided. Async version

        Args:
            viewport_api (Any): Viewport from which to retrieve/create sensor.
            sensor_types (list of omni.syntheticdata._syntheticdata.SensorType): List of sensor types to initialize.
        """
        for sensor_name in sensor_names:
            if sensor_name != "camera" and sensor_name != "pose":
                await self.sensor_helper_lib.initialize_async(viewport_api, [self.sensor_types[sensor_name]])
                await self.sensor_helper_lib.next_sensor_data_async(viewport_api)
        pass

    def get_groundtruth(self, sensor_names, viewport_api, verify_sensor_init=True, wait_for_sensor_data=0.1):
        """Get groundtruth from specified gt_sensors.

        Args:
            sensor_names (list): List of strings of sensor names. Valid sensors names: rgb, depth,
                instanceSegmentation, semanticSegmentation, boundingBox2DTight,
                boundingBox2DLoose, boundingBox3D, camera
            viewport_api (Any): Viewport from which to retrieve/create sensor.
            verify_sensor_init (bool): Additional check to verify creation and initialization of sensors.
            wait_for_sensor_data (float): Additional time to sleep before returning ground truth so  are correctly filled. Default is 0.1 seconds

        Returns:
            Dict of sensor outputs
        """
        if wait_for_sensor_data > 0:
            time.sleep(wait_for_sensor_data)

        # Create and initialize sensors
        if verify_sensor_init:
            loop = asyncio.get_event_loop()
            if loop and loop.is_running():
                carb.log_warn("Set verify_sensor_init to false if running with asyncio")
                pass
            else:
                self.initialize(sensor_names, viewport_api)

        gt = {}
        sensor_state = {}
        # Process non-RT-only sensors
        for sensor in sensor_names:
            if sensor not in ["camera", "pose"]:
                if sensor == "instanceSegmentation":
                    gt[sensor] = self.sensor_helpers[sensor](viewport_api, parsed=True, return_mapping=True)
                elif sensor == "boundingBox3D":
                    gt[sensor] = self.sensor_helpers[sensor](viewport_api, parsed=True, return_corners=True)
                else:
                    gt[sensor] = self.sensor_helpers[sensor](viewport_api)
                self.sensor_helper_lib.create_or_retrieve_sensor(viewport_api, self.sensor_types[sensor])
                # sensors are always initialized after they are created
                sensor_state[sensor] = True
            elif sensor == "pose":
                gt[sensor] = self.sensor_helpers[sensor]()
            else:
                gt[sensor] = self.sensor_helpers[sensor](viewport_api)
        gt["state"] = sensor_state

        return gt

    def get_semantic_ids(self, semantic_data: list = [[]]) -> typing.List[int]:
        """Returns unique id's for a semantic image

        Args:
            semantic_data (list, optional): Semantic Image. Defaults to [[]].

        Returns:
            typing.List[int]: List of unique semantic IDs in image
        """
        return list(np.unique(semantic_data))

    def get_semantic_id_map(self, semantic_labels: list = []) -> dict:
        """
        [Deprecated] Get map of semantic ID from label
        """
        warnings.warn(
            "This function is deprecated, use omni.replicator annotators to access semantic data", DeprecationWarning
        )
        return {}

    def get_semantic_label_map(self, semantic_ids: list = []) -> dict:
        """
        [Deprecated] Get map of semantic label from ID
        """
        warnings.warn(
            "This function is deprecated, use omni.replicator annotators to access semantic data", DeprecationWarning
        )
        return {}

    def get_mapped_semantic_data(
        self, semantic_data: list = [[]], user_semantic_label_map: dict = {}, remap_using_base_class=False
    ) -> dict:
        """[Deprecated] Map semantic segmentation data to IDs specified by user"""
        warnings.warn(
            "This function is deprecated, use omni.replicator annotators to access semantic data", DeprecationWarning
        )
        return {}
