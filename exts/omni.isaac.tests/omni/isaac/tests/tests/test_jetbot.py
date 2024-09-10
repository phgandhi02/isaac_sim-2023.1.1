# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import time

import carb
import carb.tokens
import numpy as np
import omni.graph.core as og

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils

from .robot_helpers import init_robot_sim, setup_robot_og


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestJetBot(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in jetbot (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        (result, error) = await open_stage_async(self.usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        await omni.kit.app.get_app().next_update_async()

        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path, "left_wheel_joint", "right_wheel_joint", "/jetbot", 0.0335, 0.118
        )

        pass

    # After running each test
    async def tearDown(self):
        self.my_world.stop()
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self):

        delete_prim("/ActionGraph")
        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        # get the dofbot
        self.ar = self.dc.get_articulation("/jetbot")
        self.chassis = self.dc.get_articulation_root_body(self.ar)
        self.starting_pos = np.array(self.dc.get_rigid_body_pose(self.chassis).p)

        self.wheel_left = self.dc.find_articulation_dof(self.ar, "left_wheel_joint")
        self.wheel_right = self.dc.find_articulation_dof(self.ar, "right_wheel_joint")

        # move the jetbot
        self.dc.set_dof_velocity_target(self.wheel_left, 1.0)
        self.dc.set_dof_velocity_target(self.wheel_right, 1.0)

        await dc_utils.simulate(1, self.dc, self.ar)

        self.current_pos = np.array(self.dc.get_rigid_body_pose(self.chassis).p)

        delta = np.linalg.norm(self.current_pos - self.starting_pos)
        print("Diff is ", delta)
        self.assertTrue(delta > 0.02)

        pass

    # general, slowly building up speed testcase
    # note, jetbot cannot exceed 0.42 m/s
    async def test_accel(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/jetbot")

        for x in range(1, 5):
            forward_velocity = x * 0.10
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            for i in range(15):
                await omni.kit.app.get_app().next_update_async()
            print(x, forward_velocity, og.DataView.get(odom_velocity)[0])
            if og.DataView.get(odom_ang_vel)[2] > 0.8:
                print("spinning out of control, linear velocity: " + str(forward_velocity))
                self.my_world.stop()
            else:
                self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=1e-1)
            await omni.kit.app.get_app().next_update_async()

        self.my_world.stop()

        pass

    # braking from different init speeds
    async def test_brake(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/jetbot")
        for x in range(1, 5):
            self.my_world.play()
            await omni.kit.app.get_app().next_update_async()
            forward_velocity = x * 0.10
            angular_velocity = x * 0.10
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )
            for j in range(30):
                await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            for j in range(10):
                await omni.kit.app.get_app().next_update_async()
            print(x, forward_velocity, og.DataView.get(odom_velocity)[0])
            print(x, angular_velocity, og.DataView.get(odom_ang_vel)[2])

            self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], 0.0, delta=5e-1)
            self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], 0.0, delta=5e-1)

            self.my_world.stop()
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_spin(self):
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        for x in range(1, 6):
            # Start Simulation and wait
            self.my_world.play()
            await omni.kit.app.get_app().next_update_async()

            await init_robot_sim(self.dc, "/jetbot")

            angular_velocity = 0.6 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )

            # wait until const velocity reached
            for i in range(30):
                await omni.kit.app.get_app().next_update_async()

            curr_ang_vel = float(og.DataView.get(odom_ang_vel)[2])
            self.assertAlmostEqual(curr_ang_vel, angular_velocity, delta=2e-1)

        self.my_world.stop()

        pass

    # go in circle
    async def test_circle(self):
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)
        odom_position = og.Controller.attribute("outputs:position", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/jetbot")
        forward_velocity = -0.1
        angular_velocity = -0.5
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity
        )
        for j in range(782):
            await omni.kit.app.get_app().next_update_async()
        self.assertAlmostEqual(og.DataView.get(odom_position)[0], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_position)[1], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], angular_velocity, delta=5e-2)

        await omni.kit.app.get_app().next_update_async()

        pass

    # corner case: quick accel from zero
    # async def test_jetbot_accel_drop_reset(self):
    #     odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
    #     odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

    #     # Start Simulation and wait
    #     self.my_world.play()
    #     await omni.kit.app.get_app().next_update_async()

    #     await init_robot_sim(self.dc, "/jetbot")
    #     l_wheel = self.dc.get_rigid_body("/jetbot/left_wheel")

    #     forward_velocity = 0.2
    #     og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)

    #     # wait until const velocity reached
    #     for i in range(50):
    #         await omni.kit.app.get_app().next_update_async()

    #     curr_t = 0
    #     for i in range(1600):
    #         if i - curr_t >= 200:
    #             self.my_world.stop()
    #             og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0)
    #             await omni.kit.app.get_app().next_update_async()
    #             curr_t = i
    #             self.my_world.play()
    #             await omni.kit.app.get_app().next_update_async()

    #             await init_robot_sim(self.dc, "/jetbot")
    #             # l_wheel = self.dc.get_rigid_body("/jetbot/left_wheel")

    #             # wait until const velocity reached
    #             for j in range(100):
    #                 await omni.kit.app.get_app().next_update_async()

    #             forward_velocity += 0.2
    #             og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
    #                 forward_velocity
    #             )

    #             print("forward velocity: " + str(forward_velocity))

    #             # wait until const velocity reached
    #             for j in range(100):
    #                 await omni.kit.app.get_app().next_update_async()

    #         if og.DataView.get(odom_ang_vel)[2] > 0.8:
    #             print("spinning out of control!")
    #             print("linear velocity: " + str(forward_velocity))
    #             self.my_world.stop()

    #         else:
    #             curr_vel = float(og.DataView.get(odom_velocity)[0])
    #             self.assertAlmostEqual(curr_vel, forward_velocity, delta=5e-1)
    #             self.assertAlmostEqual(
    #                 curr_vel, (self.dc.get_rigid_body_angular_velocity(l_wheel)[1]) * 0.0325, delta=0.25
    #             )
    #             print(self.dc.get_rigid_body_angular_velocity(l_wheel)[1] * 0.0325)
    #             print("correct forward velocity: " + str(forward_velocity))
    #         await omni.kit.app.get_app().next_update_async()

    #     self.my_world.stop()

    #     pass
