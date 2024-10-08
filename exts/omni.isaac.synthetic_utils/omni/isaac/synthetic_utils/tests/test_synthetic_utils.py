# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import copy
import os
import random

import carb
import carb.tokens
import numpy as np
import omni.kit.commands
import omni.kit.test
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import set_stage_up_axis

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from omni.isaac.core.utils.viewports import set_camera_view

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.synthetic_utils.writers import KittiWriter, NumpyWriter
from omni.kit.viewport.utility import get_active_viewport
from omni.physx.scripts.physicsUtils import add_ground_plane
from omni.syntheticdata.tests.utils import add_semantics
from pxr import Gf, UsdGeom, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSyntheticUtils(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._physics_rate = 60
        set_stage_up_axis("z")
        PhysicsContext(physics_dt=1.0 / self._physics_rate)
        self._time_step = 1.0 / self._physics_rate
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        carb.settings.get_settings().set("/app/asyncRendering", False)
        carb.settings.get_settings().set("/app/hydraEngine/waitIdle", True)
        carb.settings.get_settings().set("/rtx/hydra/enableSemanticSchema", True)
        await omni.kit.app.get_app().next_update_async()

        # Start Simulation and wait
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport_api = get_active_viewport()
        self._usd_context = omni.usd.get_context()
        self._sd_helper = SyntheticDataHelper()
        self._stage = self._usd_context.get_stage()
        self._camera_path = "/Camera"
        camera = self._stage.DefinePrim(self._camera_path, "Camera")
        self._viewport_api.set_active_camera(self._camera_path)

        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def initialize_sensors(self):
        # Initialize syntheticdata sensors
        await omni.kit.app.get_app().next_update_async()
        await self._sd_helper.initialize_async(
            [
                "rgb",
                "depth",
                "instanceSegmentation",
                "semanticSegmentation",
                "boundingBox2DTight",
                "boundingBox2DLoose",
                "boundingBox3D",
            ],
            self._viewport_api,
        )
        await omni.kit.app.get_app().next_update_async()

    # Acquire a copy of the ground truth.
    def get_groundtruth(self):
        gt = self._sd_helper.get_groundtruth(
            [
                "rgb",
                "depthLinear",
                "boundingBox2DTight",
                "boundingBox2DLoose",
                "instanceSegmentation",
                "semanticSegmentation",
                "boundingBox3D",
                "camera",
                "pose",
            ],
            self._viewport_api,
            verify_sensor_init=False,
        )
        return copy.deepcopy(gt)

    async def load_robot_scene(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_usd = assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd"

        add_ground_plane(self._stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.25), Gf.Vec3f(1.0))

        # setup high-level robot prim
        self.prim = self._stage.DefinePrim("/robot", "Xform")
        self.prim.GetReferences().AddReference(robot_usd)
        add_semantics(self.prim, "robot")
        rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=self.prim.GetPath(),
            old_transform_matrix=None,
            new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(0, -0.64, 0)),
        )

        # setup scene camera
        set_camera_view([3.00, 3.0, 3.00], [0, -0.64, 0], self._camera_path, self._viewport_api)
        await self.initialize_sensors()

    # Unit test for sensor groundtruth
    async def test_groundtruth(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.syntheticdata.sensors.next_sensor_data_async(self._viewport_api)
        gt = self.get_groundtruth()
        # Validate Depth groundtruth
        gt_depth = gt["depthLinear"]
        self.assertAlmostEqual(np.min(gt_depth), 5.11157, delta=0.1)
        self.assertAlmostEqual(np.max(gt_depth), 7.4313293, delta=0.1)
        # Validate 2D BBox groundtruth
        gt_bbox2d = gt["boundingBox2DTight"]
        self.assertEqual(len(gt_bbox2d), 1)
        self.assertAlmostEqual(gt_bbox2d[0][6], 432, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][7], 138, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][8], 844, delta=2)
        self.assertAlmostEqual(gt_bbox2d[0][9], 542, delta=2)
        # Validate semantic segmentation groundtruth - 0 (unlabeled) and 1 (robot)
        gt_semantic = gt["semanticSegmentation"]
        self.assertEqual(len(np.unique(gt_semantic)), 4)
        # Validate 3D BBox groundtruth
        gt_bbox3d = gt["boundingBox3D"]
        self.assertEqual(len(gt_bbox3d), 1)
        self.assertAlmostEqual(gt_bbox3d[0][6], -0.43041847, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][7], -0.31312422, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][8], -0.25173292, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][9], 0.24220554, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][10], 0.3131649, delta=0.01)
        self.assertAlmostEqual(gt_bbox3d[0][11], 0.4119104, delta=0.01)
        # Validate camera groundtruth - position, fov, focal length, aperature
        gt_camera = gt["camera"]
        gt_camera_trans = gt_camera["pose"][3, :3]
        self.assertAlmostEqual(gt_camera_trans[0], 3.000, delta=0.001)
        self.assertAlmostEqual(gt_camera_trans[1], 3.000, delta=0.001)
        self.assertAlmostEqual(gt_camera_trans[2], 3.000, delta=0.001)
        self.assertEqual(gt_camera["resolution"]["width"], 1280)
        self.assertEqual(gt_camera["resolution"]["height"], 720)
        self.assertAlmostEqual(gt_camera["fov"], 0.4131223226073451, 1e-5)
        self.assertAlmostEqual(gt_camera["focal_length"], 50.0, 1e-5)
        self.assertAlmostEqual(gt_camera["horizontal_aperture"], 20.954999923706055, 1e-2)
        # Validate pose groundtruth - prim path, semantic label, position
        gt_pose = gt["pose"]
        self.assertEqual(len(gt_pose), 1)
        self.assertEqual(gt_pose[0][0], "/robot")
        self.assertEqual(gt_pose[0][2], "robot")
        gt_pose_trans = (gt_pose[0])[3][3, :3]
        self.assertAlmostEqual(gt_pose_trans[0], 0.0, delta=0.001)
        self.assertAlmostEqual(gt_pose_trans[1], -0.640, delta=0.001)
        self.assertAlmostEqual(gt_pose_trans[2], 0.0, delta=0.001)
        pass

    # Unit test for data writer
    async def test_writer(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        # Setting up config for writer
        sensor_settings = {}
        sensor_settings_viewport = {"rgb": {"enabled": True}}
        viewport_name = viewport_window.title
        sensor_settings[viewport_name] = copy.deepcopy(sensor_settings_viewport)
        # Initialize data writer
        output_folder = os.getcwd() + "/output"
        data_writer = NumpyWriter(output_folder, 4, 100, sensor_settings)
        data_writer.start_threads()
        # Get rgb groundtruth
        gt = self._sd_helper.get_groundtruth(["rgb"], self._viewport_api, verify_sensor_init=False)
        # Write rgb groundtruth
        image_id = 1
        groundtruth = {"METADATA": {"image_id": str(image_id), "viewport_name": viewport_name}, "DATA": {}}
        groundtruth["DATA"]["RGB"] = gt["rgb"]
        data_writer.q.put(groundtruth)
        # Validate output file
        output_file_path = os.path.join(output_folder, viewport_name, "rgb", str(image_id) + ".png")
        data_writer.stop_threads()
        await asyncio.sleep(0.1)
        self.assertEqual(os.path.isfile(output_file_path), True)
        pass

        # Unit test for data writer

    async def test_kitti_writer(self):
        await self.load_robot_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        # Setting up config for writer
        sensor_settings = {}
        sensor_settings_viewport = {"rgb": {"enabled": True}}
        viewport_name = viewport_window.title
        sensor_settings[viewport_name] = copy.deepcopy(sensor_settings_viewport)
        # Initialize data writer
        output_folder_tight = os.getcwd() + "/kitti_tight"
        output_folder_loose = os.getcwd() + "/kitti_loose"
        data_writer_tight = KittiWriter(
            output_folder_tight, 4, 100, train_size=1, classes="robot", bbox_type="BBOX2DTIGHT"
        )
        data_writer_tight.start_threads()
        data_writer_loose = KittiWriter(
            output_folder_loose, 4, 100, train_size=1, classes="robot", bbox_type="BBOX2DLOOSE"
        )
        data_writer_loose.start_threads()
        # Get rgb groundtruth
        gt = self._sd_helper.get_groundtruth(
            ["rgb", "boundingBox2DTight", "boundingBox2DLoose"], self._viewport_api, verify_sensor_init=False
        )
        # Write rgb groundtruth
        image_id = 0
        groundtruth = {
            "METADATA": {
                "image_id": str(image_id),
                "viewport_name": viewport_name,
                "BBOX2DTIGHT": {},
                "BBOX2DLOOSE": {},
            },
            "DATA": {},
        }
        image = gt["rgb"]
        groundtruth["DATA"]["RGB"] = image
        groundtruth["DATA"]["BBOX2DTIGHT"] = gt["boundingBox2DTight"]
        groundtruth["METADATA"]["BBOX2DTIGHT"]["WIDTH"] = image.shape[1]
        groundtruth["METADATA"]["BBOX2DTIGHT"]["HEIGHT"] = image.shape[0]

        groundtruth["DATA"]["BBOX2DLOOSE"] = gt["boundingBox2DLoose"]
        groundtruth["METADATA"]["BBOX2DLOOSE"]["WIDTH"] = image.shape[1]
        groundtruth["METADATA"]["BBOX2DLOOSE"]["HEIGHT"] = image.shape[0]
        for f in range(2):
            groundtruth["METADATA"]["image_id"] = image_id
            data_writer_tight.q.put(copy.deepcopy(groundtruth))
            data_writer_loose.q.put(copy.deepcopy(groundtruth))
            image_id = image_id + 1

        # Validate output file
        data_writer_tight.stop_threads()
        data_writer_loose.stop_threads()
        await asyncio.sleep(0.1)

        for output_folder in [output_folder_tight, output_folder_loose]:
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/training/image_2", str(0) + ".png")), True)
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/training/label_2", str(0) + ".txt")), True)
            self.assertEqual(os.path.isfile(os.path.join(output_folder + "/testing/image_2", str(1) + ".png")), True)
        pass
