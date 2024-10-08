import os
import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
from omni.graph.core.tests.omnigraph_test_utils import _TestGraphAndNode
from omni.graph.core.tests.omnigraph_test_utils import _test_clear_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_setup_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_verify_scene


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.range_sensor.ogn.OgnIsaacReadLidarBeamsDatabase import OgnIsaacReadLidarBeamsDatabase
        test_file_name = "OgnIsaacReadLidarBeamsTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_range_sensor_IsaacReadLidarBeams")
        database = OgnIsaacReadLidarBeamsDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:lidarPrim"))
        attribute = test_node.get_attribute("inputs:lidarPrim")
        db_value = database.inputs.lidarPrim

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuthRange"))
        attribute = test_node.get_attribute("outputs:azimuthRange")
        db_value = database.outputs.azimuthRange

        self.assertTrue(test_node.get_attribute_exists("outputs:beamTimeData"))
        attribute = test_node.get_attribute("outputs:beamTimeData")
        db_value = database.outputs.beamTimeData

        self.assertTrue(test_node.get_attribute_exists("outputs:depthRange"))
        attribute = test_node.get_attribute("outputs:depthRange")
        db_value = database.outputs.depthRange

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:horizontalFov"))
        attribute = test_node.get_attribute("outputs:horizontalFov")
        db_value = database.outputs.horizontalFov

        self.assertTrue(test_node.get_attribute_exists("outputs:horizontalResolution"))
        attribute = test_node.get_attribute("outputs:horizontalResolution")
        db_value = database.outputs.horizontalResolution

        self.assertTrue(test_node.get_attribute_exists("outputs:intensitiesData"))
        attribute = test_node.get_attribute("outputs:intensitiesData")
        db_value = database.outputs.intensitiesData

        self.assertTrue(test_node.get_attribute_exists("outputs:linearDepthData"))
        attribute = test_node.get_attribute("outputs:linearDepthData")
        db_value = database.outputs.linearDepthData

        self.assertTrue(test_node.get_attribute_exists("outputs:numCols"))
        attribute = test_node.get_attribute("outputs:numCols")
        db_value = database.outputs.numCols

        self.assertTrue(test_node.get_attribute_exists("outputs:numRows"))
        attribute = test_node.get_attribute("outputs:numRows")
        db_value = database.outputs.numRows

        self.assertTrue(test_node.get_attribute_exists("outputs:rotationRate"))
        attribute = test_node.get_attribute("outputs:rotationRate")
        db_value = database.outputs.rotationRate

        self.assertTrue(test_node.get_attribute_exists("outputs:verticalFov"))
        attribute = test_node.get_attribute("outputs:verticalFov")
        db_value = database.outputs.verticalFov

        self.assertTrue(test_node.get_attribute_exists("outputs:verticalResolution"))
        attribute = test_node.get_attribute("outputs:verticalResolution")
        db_value = database.outputs.verticalResolution

        self.assertTrue(test_node.get_attribute_exists("outputs:zenithRange"))
        attribute = test_node.get_attribute("outputs:zenithRange")
        db_value = database.outputs.zenithRange
