import os
import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
from omni.graph.core.tests.omnigraph_test_utils import _TestGraphAndNode
from omni.graph.core.tests.omnigraph_test_utils import _test_clear_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_setup_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_verify_scene


class TestOgn(ogts.OmniGraphTestCase):

    TEST_DATA = [
        {
            'inputs': [
                ['inputs:renderProductPath', "/Render/RenderProduct_omni_kit_widget_viewport_ViewportTexture_0", False],
            ],
            'outputs': [
                ['outputs:focalLength', 18.14756202697754, False],
            ],
            'setup': {'create_nodes': [['TestNode', 'omni.isaac.core_nodes.IsaacReadCameraInfo']]}        },
    ]

    async def test_generated(self):
        test_info = _TestGraphAndNode()
        controller = og.Controller()
        for i, test_run in enumerate(self.TEST_DATA):
            await _test_clear_scene(self, test_run)
            test_info = await _test_setup_scene(self, controller, "/TestGraph", "TestNode_omni_isaac_core_nodes_IsaacReadCameraInfo", "omni.isaac.core_nodes.IsaacReadCameraInfo", test_run, test_info)
            await controller.evaluate(test_info.graph)
            _test_verify_scene(self, controller, test_run, test_info, f"omni.isaac.core_nodes.IsaacReadCameraInfo User test case #{i+1}")

    async def test_vectorized_generated(self):
        test_info = _TestGraphAndNode()
        controller = og.Controller()
        for i, test_run in enumerate(self.TEST_DATA):
            await _test_clear_scene(self, test_run)
            test_info = await _test_setup_scene(self, controller, "/TestGraph", "TestNode_omni_isaac_core_nodes_IsaacReadCameraInfo","omni.isaac.core_nodes.IsaacReadCameraInfo", test_run, test_info, 16)
            await controller.evaluate(test_info.graph)
            _test_verify_scene(self, controller, test_run, test_info, f"omni.isaac.core_nodes.IsaacReadCameraInfo User test case #{i+1}", 16)

    async def test_data_access(self):
        from omni.isaac.core_nodes.ogn.OgnIsaacReadCameraInfoDatabase import OgnIsaacReadCameraInfoDatabase
        test_file_name = "OgnIsaacReadCameraInfoTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_core_nodes_IsaacReadCameraInfo")
        database = OgnIsaacReadCameraInfoDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:renderProductPath"))
        attribute = test_node.get_attribute("inputs:renderProductPath")
        db_value = database.inputs.renderProductPath
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:viewport"))
        attribute = test_node.get_attribute("inputs:viewport")
        db_value = database.inputs.viewport
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:cameraFisheyeParams"))
        attribute = test_node.get_attribute("outputs:cameraFisheyeParams")
        db_value = database.outputs.cameraFisheyeParams

        self.assertTrue(test_node.get_attribute_exists("outputs:focalLength"))
        attribute = test_node.get_attribute("outputs:focalLength")
        db_value = database.outputs.focalLength

        self.assertTrue(test_node.get_attribute_exists("outputs:height"))
        attribute = test_node.get_attribute("outputs:height")
        db_value = database.outputs.height

        self.assertTrue(test_node.get_attribute_exists("outputs:horizontalAperture"))
        attribute = test_node.get_attribute("outputs:horizontalAperture")
        db_value = database.outputs.horizontalAperture

        self.assertTrue(test_node.get_attribute_exists("outputs:horizontalOffset"))
        attribute = test_node.get_attribute("outputs:horizontalOffset")
        db_value = database.outputs.horizontalOffset

        self.assertTrue(test_node.get_attribute_exists("outputs:physicalDistortionCoefficients"))
        attribute = test_node.get_attribute("outputs:physicalDistortionCoefficients")
        db_value = database.outputs.physicalDistortionCoefficients

        self.assertTrue(test_node.get_attribute_exists("outputs:physicalDistortionModel"))
        attribute = test_node.get_attribute("outputs:physicalDistortionModel")
        db_value = database.outputs.physicalDistortionModel

        self.assertTrue(test_node.get_attribute_exists("outputs:projectionType"))
        attribute = test_node.get_attribute("outputs:projectionType")
        db_value = database.outputs.projectionType

        self.assertTrue(test_node.get_attribute_exists("outputs:verticalAperture"))
        attribute = test_node.get_attribute("outputs:verticalAperture")
        db_value = database.outputs.verticalAperture

        self.assertTrue(test_node.get_attribute_exists("outputs:verticalOffset"))
        attribute = test_node.get_attribute("outputs:verticalOffset")
        db_value = database.outputs.verticalOffset

        self.assertTrue(test_node.get_attribute_exists("outputs:width"))
        attribute = test_node.get_attribute("outputs:width")
        db_value = database.outputs.width
