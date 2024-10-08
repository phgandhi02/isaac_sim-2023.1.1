import carb
import omni
from omni.physx.scripts import utils, physicsUtils
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory, PhysicsKitStageAsyncTestCase
import omni.physx.bindings._physx as physx_settings
import carb.settings
import unittest


class PhysxInterfaceSimulationEventsTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.STOPPED):
            self._stoppedEventCount = self._stoppedEventCount + 1

    async def test_physics_sim_event_stopped_on_close(self):
        stage = await self.new_stage()

        physxInterface = get_physx_interface()
        for j in range(2):
            # 0: close stage after simulation was started -> expect event
            # 1: close stage after simulation ended       -> expect no extra event

            self._stoppedEventCount = 0
            simulationEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
                self._on_simulation_event
            )

            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
            UsdGeom.SetStageMetersPerUnit(stage, 1)

            scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")            
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1.0, 0))
            scene.CreateGravityMagnitudeAttr(10.0)

            sphereActorPath = "/SphereActor"
            radius = 0.1
            color = Gf.Vec3f(255.0, 255.0, 255.0)
            physicsUtils.add_rigid_sphere(
                stage,
                sphereActorPath,
                radius,
                Gf.Vec3f(0.0, 1.0, 0.0),
                Gf.Quatf(1.0),
                color,
                1000.0,
                Gf.Vec3f(0.0),
                Gf.Vec3f(0.0),
            )

            timeStep = 1.0 / 60.0
            physxInterface.start_simulation()
            for i in range(10):
                physxInterface.update_simulation(timeStep, timeStep)

            if j == 1:
                physxInterface.reset_simulation()
                self.assertTrue(self._stoppedEventCount == 1)

            stage = await self.new_stage()
                
            # A) if the simulation was not ended, an event should get sent on close
            # B) if the simulation was ended, there should not be another event
            self.assertTrue(self._stoppedEventCount == 1)

            simulationEventSubcription = None

    async def test_physics_subscription_default_scene(self):
            await omni.physxtests.utils.new_stage_setup()
            # await omni.usd.get_context().new_stage_async()
            # await omni.kit.app.get_app().next_update_async()
            timeline = omni.timeline.get_timeline_interface()
            self.check_dt = 0.0  # set this to zero to start

            def on_update(dt):
                self.check_dt = dt

            async_update_loop = 3
            sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_update)
            timeline.play()
            for _ in range(async_update_loop):
                await omni.kit.app.get_app().next_update_async()            
            self.assertNotEqual(self.check_dt, 0.0)

            # check passes
            timeline.stop()
            for _ in range(async_update_loop):
                await omni.kit.app.get_app().next_update_async()            
            self.check_dt = 0.0  # reset this to zero to see if it changes after a stop/play
            timeline.play()
            for _ in range(async_update_loop):
                await omni.kit.app.get_app().next_update_async()            
            self.assertNotEqual(self.check_dt, 0.0)
            # check fails
            sub = None

    async def test_physics_step_subscription_simulation_iface(self):
            settings = carb.settings.acquire_settings_interface()
            saved_num_threads = settings.get_as_int(physx_settings.SETTING_NUM_THREADS)
            settings.set(physx_settings.SETTING_NUM_THREADS, 4)
            stage = await self.new_stage()
            self.check_dt = 0.0  # set this to zero to start

            def on_update(dt):
                self.check_dt = dt
                
            scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")

            sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_update)

            omni.physx.get_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
            omni.physx.get_physx_simulation_interface().fetch_results()
            
            self.assertNotEqual(self.check_dt, 0.0)

            # check fails
            sub = None
            settings.set(physx_settings.SETTING_NUM_THREADS, saved_num_threads)

class PhysxInterfaceSimulationEventsTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        self._frame_count = 0
        self._sim_time = 0.0
        self._timeStepsPerSecond = 60
        self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        isregistry = carb.settings.get_settings()
        self._min_frame_rate = isregistry.get(physx_settings.SETTING_MIN_FRAME_RATE)
        isregistry.set(physx_settings.SETTING_MIN_FRAME_RATE, 30)

    async def tearDown(self):
        self._physics_update_sub = None
        carb.settings.get_settings().set(physx_settings.SETTING_MIN_FRAME_RATE, self._min_frame_rate)
        await super().tearDown()

    def _on_physics_step(self, dt):
        self._frame_count += 1        
        self._sim_time += dt

    async def test_physics_async_sim_stepping(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1.0, 0))
        scene.CreateGravityMagnitudeAttr(0.0)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateTimeStepsPerSecondAttr().Set(self._timeStepsPerSecond)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())

        velocity = 0.987527
        sphere_prim = physicsUtils.add_rigid_sphere(
            stage,
            "/SphereActor",
            lin_velocity=Gf.Vec3f(velocity, 0.0, 0.0),
        )
        num_runs = 3
        num_frames = 50
        for _ in range(num_runs):
            omni.timeline.get_timeline_interface().play()
            while(self._frame_count < num_frames):
                await omni.kit.app.get_app().next_update_async()
            # stop, check for reset and prep for next run
            omni.timeline.get_timeline_interface().stop()

            pos_act = sphere_prim.GetAttribute("xformOp:translate").Get()
            sim_dist_pred = self._sim_time * velocity
            # allow an error of up to two timesteps that may arrive in async between stop and getting the attribute:
            tol = 2.0 / self._timeStepsPerSecond * velocity * (1.0 + 1e-5)
            self.assertGreaterEqual(pos_act[0] + tol, sim_dist_pred)

            await omni.kit.app.get_app().next_update_async()
            pos_act_stop = sphere_prim.GetAttribute("xformOp:translate").Get()
            self.assertEqual(pos_act_stop[0], 0.0)
            self._frame_count = 0
            self._sim_time = 0.0


    # test to ensure that async sim scenes fire step callbacks at the same rate as the simulation steps
    # since our robotics demos rely on this for control.
    async def test_physics_async_sim_substepping(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1.0, 0))
        scene.CreateGravityMagnitudeAttr(0.0)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        self._timeStepsPerSecond = 1.0e3  # choose very small to get substepping between render frames
        sim_dt = 1.0 / self._timeStepsPerSecond
        expectedSubsteps = 10.0  # allow many substeps
        physxSceneAPI.CreateTimeStepsPerSecondAttr().Set(self._timeStepsPerSecond)
        # set min framerate to get expectedSubsteps:
        carb.settings.get_settings().set(physx_settings.SETTING_MIN_FRAME_RATE, round(self._timeStepsPerSecond / expectedSubsteps))

        utils.set_physics_scene_asyncsimrender(scene.GetPrim())

        velocity = 10.0

        # add an actor to track simulation progress
        sphere_prim = physicsUtils.add_rigid_sphere(
            stage,
            "/SphereActor",
            lin_velocity=Gf.Vec3f(velocity, 0.0, 0.0),
        )
        num_runs = 3
        num_updates = 10
        for _ in range(num_runs):
            self._sim_time = 0.0
            omni.timeline.get_timeline_interface().play()
            for i in range(num_updates):                
                print("Kit async update")
                await omni.kit.app.get_app().next_update_async()                

            # we get less steps make sure the changes are written                
            get_physx_simulation_interface().simulate(0.1, 0.1)
            get_physx_simulation_interface().fetch_results()
            omni.timeline.get_timeline_interface().stop()

            # sim time is updated in test fixture step callback            
            pos_pred = self._sim_time * velocity
            pos_act = sphere_prim.GetAttribute("xformOp:translate").Get()
            tol = expectedSubsteps * sim_dt * velocity
            # there will be a mismatch if we have a big discrepancy between number of callbacks and sim steps
            self.assertAlmostEqual(pos_pred, pos_act[0], delta=tol)
            await omni.kit.app.get_app().next_update_async()
