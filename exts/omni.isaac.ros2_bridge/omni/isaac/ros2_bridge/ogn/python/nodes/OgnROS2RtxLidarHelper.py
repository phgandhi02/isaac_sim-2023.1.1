# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import traceback

import carb
import omni
import omni.graph.core as og
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.replicator.core as rep
import omni.syntheticdata
from omni.isaac.core.utils.render_product import get_camera_prim_path
from omni.isaac.core_nodes import BaseWriterNode, WriterRequest
from pxr import Usd, UsdGeom


class OgnROS2RtxLidarHelperInternalState(BaseWriterNode):
    def __init__(self):
        self.viewport = None
        self.viewport_name = ""
        self.resetSimulationTimeOnStop = False
        super().__init__(initialize=False)

    def post_attach(self, writer, render_product):
        try:
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except:
            pass


class OgnROS2RtxLidarHelper:
    @staticmethod
    def internal_state():
        return OgnROS2RtxLidarHelperInternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.enabled is False:
            if db.internal_state.initialized is False:
                return True
            else:
                db.internal_state.custom_reset()
                return True

        if db.internal_state.initialized is False:
            db.internal_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            keys = og.Controller.Keys
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                render_product_path = db.inputs.renderProductPath
                if not render_product_path:
                    carb.log_warn("Render product not valid")
                    db.internal_state.initialized = False
                    return False
                if stage.GetPrimAtPath(render_product_path) is None:
                    # Invalid Render Product Path
                    carb.log_warn("Render product not created yet, retrying on next call")
                    db.internal_state.initialized = False
                    return False
                else:
                    prim = stage.GetPrimAtPath(get_camera_prim_path(render_product_path))
                    if prim.IsA(UsdGeom.Camera):
                        if prim.HasAPI(IsaacSensorSchema.IsaacRtxLidarSensorAPI):
                            db.internal_state.render_product_path = render_product_path
                            db.internal_state.sensor = "lidar"
                        else:
                            db.internal_state.sensor = None

                if db.internal_state.sensor is None:
                    carb.log_warn("Active camera for Render product is not an RTX Lidar")
                    db.internal_state.initialized = False
                    return False

                db.internal_state.render_product_path = render_product_path
                sensor_type = db.inputs.type
                db.internal_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
                writer = None
                try:
                    if sensor_type == "laser_scan":
                        if db.inputs.fullScan:
                            carb.log_warn("fullScan does not have an effect with the laser_scan setting")
                        writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")

                    elif sensor_type == "point_cloud":
                        if db.inputs.fullScan:
                            writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloudBuffer")
                        else:
                            writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")

                    else:
                        carb.log_error("type is not supported")
                        db.internal_state.initialized = False
                        return False
                    if writer is not None:
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )
                        db.internal_state.append_writer(writer)
                    db.internal_state.attach_writers(render_product_path)
                except Exception as e:
                    print(traceback.format_exc())
                    pass
        else:
            return True

    @staticmethod
    def release(node):
        try:
            state = OgnROS2RtxLidarHelperInternalState.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
