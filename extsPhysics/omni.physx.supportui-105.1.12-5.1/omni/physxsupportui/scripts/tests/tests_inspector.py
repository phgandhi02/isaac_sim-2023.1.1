import carb
import omni.usd
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema
from omni.physxtests import utils
import omni.ui as ui
import omni.kit.ui_test as ui_test
from omni.physxtestsvisual.utils import TestCase
from omni.kit.viewport.utility import frame_viewport_selection, get_active_viewport
from ..utils import ui_wait
import unittest

class PhysicsInspectorTests(TestCase):
    async def setUp(self):
        await super().setUp()
        self._stage = await utils.new_stage_setup(False)
        self._settings = carb.settings.get_settings()
        self._prev_inspector_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED)

    async def tearDown(self):
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, self._prev_inspector_enabled)
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
        self._stage = None
        await utils.new_stage_setup()
        
    async def test_inspector_basic(self):
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, False)
        self._create_articulation("/World", self._stage)
        scene : UsdPhysics.Scene = UsdPhysics.Scene.Define(self._stage, "/physics_scene")
        scene.CreateGravityMagnitudeAttr().Set(0.0)
        self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, True)
        await ui_wait(10)
        inspector_window = await self._select_and_focus_property_window("/World", 1)
        slider_path = "//Frame/VStack[0]/ZStack[0]/VStack[2]/ScrollingFrame[0]/ZStack[0]/TreeView[0]/ZStack[1]/FloatSlider[0]"
        slider = ui_test.find(f"{inspector_window.title}{slider_path}")
        await ui_test.emulate_mouse_move_and_click(slider.position + ui_test.Vec2(2, 10))

        joint_prim : Usd.Prim = self._stage.GetPrimAtPath("/World/articulation/revoluteJoint")
        joint_state : PhysxSchema.JointStateAPI = PhysxSchema.JointStateAPI.Get(joint_prim, "angular")
        joint_value = joint_state.GetPositionAttr().Get()
        self.assertAlmostEqual(joint_value, -90.0, delta=0.01)
        self._viewport_focus_selection()
        await ui_wait(20)
        await self.setup_viewport_test(1000, 800)
        await ui_wait(20)
        all_tests_passed = True
        all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_inspector_basic",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025
        )
        self.assertTrue(all_tests_passed)
        await self.new_stage()

    def _viewport_focus_selection(self):
        active_viewport = get_active_viewport()
        if active_viewport:
            frame_viewport_selection(active_viewport)

    async def _select_and_focus_property_window(self, path, index) -> ui.Window :
        # select prim and focus property window so the widgets are built
        omni.usd.get_context().get_selection().set_selected_prim_paths([path], False)
        await ui_wait(10)
        window = ui.Workspace.get_window(f"Physics Inspector: {path}###PhysicsInspector{index}")
        return window

    def _create_articulation(self, defaultPrimPath : str, stage : Usd.Stage):
        articulationPath = defaultPrimPath + "/articulation"
        UsdGeom.Xform.Define(stage, articulationPath)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))

        # box0 static
        boxActorPath = articulationPath + "/box0"

        position = Gf.Vec3f(0.0, 0.0, 1000.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(1.0, 0.0, 0.0)
        size = 100.0
        scale = Gf.Vec3f(0.1, 1.0, 0.1)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)            

        # Box1
        boxActorPath = articulationPath + "/box1"

        size = 100.0
        position = Gf.Vec3f(0.0, 120.0, 1000.0)
        color = Gf.Vec3f(0.0, 0.0, 1.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        scale = Gf.Vec3f(0.1, 1.0, 0.1)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

        # fixed root joint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, articulationPath + "/rootJoint")
        fixedJoint.CreateBody1Rel().SetTargets( [Sdf.Path(articulationPath + "/box0")])

        fixedJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixedJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # revolute joint
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, articulationPath + "/revoluteJoint")

        revoluteJoint.CreateAxisAttr("X")
        revoluteJoint.CreateLowerLimitAttr(-90.0)
        revoluteJoint.CreateUpperLimitAttr(90)

        revoluteJoint.CreateBody0Rel().SetTargets([articulationPath + "/box0"])
        revoluteJoint.CreateBody1Rel().SetTargets([articulationPath + "/box1"])

        revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 60.0, 0.0))
        revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -60.0, 0.0))
        revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        jointStateAPI = PhysxSchema.JointStateAPI.Apply(revoluteJoint.GetPrim(), "angular")
        jointStateAPI.CreatePositionAttr().Set(45.0)
        jointStateAPI.CreateVelocityAttr().Set(0.0)
