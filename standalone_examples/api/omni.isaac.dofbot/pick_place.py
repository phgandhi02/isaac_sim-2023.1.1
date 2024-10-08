# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.dofbot.controllers.pick_place_controller import PickPlaceController
from omni.isaac.dofbot.tasks import PickPlace

my_world = World(stage_units_in_meters=1.0)
my_task = PickPlace()
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
dofbot_name = task_params["robot_name"]["value"]
my_dofbot = my_world.scene.get_object(dofbot_name)
my_controller = PickPlaceController(
    name="pick_place_controller", gripper=my_dofbot.gripper, robot_articulation=my_dofbot
)
articulation_controller = my_dofbot.get_articulation_controller()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[dofbot_name]["joint_positions"],
            end_effector_offset=np.array([0, -0.06, 0]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)

simulation_app.close()
