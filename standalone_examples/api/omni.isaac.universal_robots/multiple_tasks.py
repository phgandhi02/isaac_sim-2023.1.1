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

import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dofbot.controllers import PickPlaceController
from omni.isaac.dofbot.tasks import PickPlace
from omni.isaac.franka.controllers.stacking_controller import StackingController as FrankaStackingController
from omni.isaac.franka.tasks import Stacking as FrankaStacking
from omni.isaac.universal_robots.controllers import StackingController as UR10StackingController
from omni.isaac.universal_robots.tasks import Stacking as UR10Stacking
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup

my_world = World(stage_units_in_meters=1.0)
tasks = []
num_of_tasks = 3

tasks.append(FrankaStacking(name="task_0", offset=np.array([0, -2, 0])))
my_world.add_task(tasks[-1])
tasks.append(UR10Stacking(name="task_1", offset=np.array([0.5, 0.5, 0])))
my_world.add_task(tasks[-1])
tasks.append(PickPlace(offset=np.array([0, -1, 0])))
my_world.add_task(tasks[-1])
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
my_kaya = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Kaya",
        name="my_kaya",
        wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
        create_robot=True,
        usd_path=kaya_asset_path,
        position=np.array([-1, 0, 0]),
    )
)
jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([-1.5, -1.5, 0]),
    )
)

my_world.reset()
robots = []
for i in range(num_of_tasks):
    task_params = tasks[i].get_params()
    robots.append(my_world.scene.get_object(task_params["robot_name"]["value"]))

controllers = []
controllers.append(
    FrankaStackingController(
        name="pick_place_controller",
        gripper=robots[0].gripper,
        robot_articulation=robots[0],
        picking_order_cube_names=tasks[0].get_cube_names(),
        robot_observation_name=robots[0].name,
    )
)
controllers[-1].reset()
controllers.append(
    UR10StackingController(
        name="pick_place_controller",
        gripper=robots[1].gripper,
        robot_articulation=robots[1],
        picking_order_cube_names=tasks[1].get_cube_names(),
        robot_observation_name=robots[1].name,
    )
)
controllers[-1].reset()
controllers.append(
    PickPlaceController(name="pick_place_controller", gripper=robots[2].gripper, robot_articulation=robots[2])
)

kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.prim_path, com_prim_path="/World/Kaya/base_link/control_offset"
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()
kaya_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

jetbot_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
pick_place_task_params = tasks[2].get_params()

articulation_controllers = []
for i in range(num_of_tasks):
    articulation_controllers.append(robots[i].get_articulation_controller())

i = 0
my_world.pause()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            controllers[0].reset()
            controllers[1].reset()
            controllers[2].reset()
            kaya_controller.reset()
            jetbot_controller.reset()
        observations = my_world.get_observations()
        actions = controllers[0].forward(observations=observations, end_effector_offset=np.array([0, 0, 0]))
        articulation_controllers[0].apply_action(actions)
        actions = controllers[1].forward(observations=observations, end_effector_offset=np.array([0, 0, 0.02]))
        articulation_controllers[1].apply_action(actions)

        actions = controllers[2].forward(
            picking_position=observations[pick_place_task_params["cube_name"]["value"]]["position"],
            placing_position=observations[pick_place_task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[pick_place_task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, -0.06, 0]),
        )
        articulation_controllers[2].apply_action(actions)
        if i >= 0 and i < 500:
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0.2, 0.0, 0.0]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.1, 0]))
        elif i >= 500 and i < 1000:
            # TODO: change with new USD
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0, 0.2, 0.0]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.0, np.pi / 10]))
        elif i >= 1000 and i < 1500:
            # TODO: change with new USD
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0, 0.0, 0.6]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.1, 0]))
        i += 1


simulation_app.close()
