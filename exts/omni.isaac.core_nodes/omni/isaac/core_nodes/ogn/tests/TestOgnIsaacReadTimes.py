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
        from omni.isaac.core_nodes.ogn.OgnIsaacReadTimesDatabase import OgnIsaacReadTimesDatabase
        test_file_name = "OgnIsaacReadTimesTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_core_nodes_IsaacReadTimes")
        database = OgnIsaacReadTimesDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:gpu"))
        attribute = test_node.get_attribute("inputs:gpu")
        db_value = database.inputs.gpu
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderResults"))
        attribute = test_node.get_attribute("inputs:renderResults")
        db_value = database.inputs.renderResults
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:durationDenominator"))
        attribute = test_node.get_attribute("outputs:durationDenominator")
        db_value = database.outputs.durationDenominator

        self.assertTrue(test_node.get_attribute_exists("outputs:durationNumerator"))
        attribute = test_node.get_attribute("outputs:durationNumerator")
        db_value = database.outputs.durationNumerator

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:externalTimeOfSimNs"))
        attribute = test_node.get_attribute("outputs:externalTimeOfSimNs")
        db_value = database.outputs.externalTimeOfSimNs

        self.assertTrue(test_node.get_attribute_exists("outputs:frameNumber"))
        attribute = test_node.get_attribute("outputs:frameNumber")
        db_value = database.outputs.frameNumber

        self.assertTrue(test_node.get_attribute_exists("outputs:rationalTimeOfSimDenominator"))
        attribute = test_node.get_attribute("outputs:rationalTimeOfSimDenominator")
        db_value = database.outputs.rationalTimeOfSimDenominator

        self.assertTrue(test_node.get_attribute_exists("outputs:rationalTimeOfSimNumerator"))
        attribute = test_node.get_attribute("outputs:rationalTimeOfSimNumerator")
        db_value = database.outputs.rationalTimeOfSimNumerator

        self.assertTrue(test_node.get_attribute_exists("outputs:sampleTimeOffsetInSimFrames"))
        attribute = test_node.get_attribute("outputs:sampleTimeOffsetInSimFrames")
        db_value = database.outputs.sampleTimeOffsetInSimFrames

        self.assertTrue(test_node.get_attribute_exists("outputs:simulationTime"))
        attribute = test_node.get_attribute("outputs:simulationTime")
        db_value = database.outputs.simulationTime

        self.assertTrue(test_node.get_attribute_exists("outputs:simulationTimeMonotonic"))
        attribute = test_node.get_attribute("outputs:simulationTimeMonotonic")
        db_value = database.outputs.simulationTimeMonotonic

        self.assertTrue(test_node.get_attribute_exists("outputs:swhFrameNumber"))
        attribute = test_node.get_attribute("outputs:swhFrameNumber")
        db_value = database.outputs.swhFrameNumber

        self.assertTrue(test_node.get_attribute_exists("outputs:systemTime"))
        attribute = test_node.get_attribute("outputs:systemTime")
        db_value = database.outputs.systemTime
