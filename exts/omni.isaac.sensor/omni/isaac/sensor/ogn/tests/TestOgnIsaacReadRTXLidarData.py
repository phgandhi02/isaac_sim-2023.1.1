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
        from omni.isaac.sensor.ogn.OgnIsaacReadRTXLidarDataDatabase import OgnIsaacReadRTXLidarDataDatabase
        test_file_name = "OgnIsaacReadRTXLidarDataTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_sensor_IsaacReadRTXLidarData")
        database = OgnIsaacReadRTXLidarDataDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:dataPtr"))
        attribute = test_node.get_attribute("inputs:dataPtr")
        db_value = database.inputs.dataPtr
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        db_value = database.inputs.exec

        self.assertTrue(test_node.get_attribute_exists("inputs:keepOnlyPositiveDistance"))
        attribute = test_node.get_attribute("inputs:keepOnlyPositiveDistance")
        db_value = database.inputs.keepOnlyPositiveDistance
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderProductPath"))
        attribute = test_node.get_attribute("inputs:renderProductPath")
        db_value = database.inputs.renderProductPath
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuths"))
        attribute = test_node.get_attribute("outputs:azimuths")
        db_value = database.outputs.azimuths

        self.assertTrue(test_node.get_attribute_exists("outputs:beamIds"))
        attribute = test_node.get_attribute("outputs:beamIds")
        db_value = database.outputs.beamIds

        self.assertTrue(test_node.get_attribute_exists("outputs:channels"))
        attribute = test_node.get_attribute("outputs:channels")
        db_value = database.outputs.channels

        self.assertTrue(test_node.get_attribute_exists("outputs:deltaTimes"))
        attribute = test_node.get_attribute("outputs:deltaTimes")
        db_value = database.outputs.deltaTimes

        self.assertTrue(test_node.get_attribute_exists("outputs:depthRange"))
        attribute = test_node.get_attribute("outputs:depthRange")
        db_value = database.outputs.depthRange

        self.assertTrue(test_node.get_attribute_exists("outputs:distances"))
        attribute = test_node.get_attribute("outputs:distances")
        db_value = database.outputs.distances

        self.assertTrue(test_node.get_attribute_exists("outputs:echos"))
        attribute = test_node.get_attribute("outputs:echos")
        db_value = database.outputs.echos

        self.assertTrue(test_node.get_attribute_exists("outputs:elevations"))
        attribute = test_node.get_attribute("outputs:elevations")
        db_value = database.outputs.elevations

        self.assertTrue(test_node.get_attribute_exists("outputs:emitterIds"))
        attribute = test_node.get_attribute("outputs:emitterIds")
        db_value = database.outputs.emitterIds

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:hitPointNormals"))
        attribute = test_node.get_attribute("outputs:hitPointNormals")
        db_value = database.outputs.hitPointNormals

        self.assertTrue(test_node.get_attribute_exists("outputs:intensities"))
        attribute = test_node.get_attribute("outputs:intensities")
        db_value = database.outputs.intensities

        self.assertTrue(test_node.get_attribute_exists("outputs:materialIds"))
        attribute = test_node.get_attribute("outputs:materialIds")
        db_value = database.outputs.materialIds

        self.assertTrue(test_node.get_attribute_exists("outputs:numBeams"))
        attribute = test_node.get_attribute("outputs:numBeams")
        db_value = database.outputs.numBeams

        self.assertTrue(test_node.get_attribute_exists("outputs:numChannels"))
        attribute = test_node.get_attribute("outputs:numChannels")
        db_value = database.outputs.numChannels

        self.assertTrue(test_node.get_attribute_exists("outputs:numEchos"))
        attribute = test_node.get_attribute("outputs:numEchos")
        db_value = database.outputs.numEchos

        self.assertTrue(test_node.get_attribute_exists("outputs:numTicks"))
        attribute = test_node.get_attribute("outputs:numTicks")
        db_value = database.outputs.numTicks

        self.assertTrue(test_node.get_attribute_exists("outputs:objectIds"))
        attribute = test_node.get_attribute("outputs:objectIds")
        db_value = database.outputs.objectIds

        self.assertTrue(test_node.get_attribute_exists("outputs:tickAzimuths"))
        attribute = test_node.get_attribute("outputs:tickAzimuths")
        db_value = database.outputs.tickAzimuths

        self.assertTrue(test_node.get_attribute_exists("outputs:tickStates"))
        attribute = test_node.get_attribute("outputs:tickStates")
        db_value = database.outputs.tickStates

        self.assertTrue(test_node.get_attribute_exists("outputs:tickTimestamps"))
        attribute = test_node.get_attribute("outputs:tickTimestamps")
        db_value = database.outputs.tickTimestamps

        self.assertTrue(test_node.get_attribute_exists("outputs:ticks"))
        attribute = test_node.get_attribute("outputs:ticks")
        db_value = database.outputs.ticks

        self.assertTrue(test_node.get_attribute_exists("outputs:velocities"))
        attribute = test_node.get_attribute("outputs:velocities")
        db_value = database.outputs.velocities
