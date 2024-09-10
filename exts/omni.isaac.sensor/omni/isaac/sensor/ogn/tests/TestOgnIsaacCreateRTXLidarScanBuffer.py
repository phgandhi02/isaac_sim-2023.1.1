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
        from omni.isaac.sensor.ogn.OgnIsaacCreateRTXLidarScanBufferDatabase import OgnIsaacCreateRTXLidarScanBufferDatabase
        test_file_name = "OgnIsaacCreateRTXLidarScanBufferTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_sensor_IsaacCreateRTXLidarScanBuffer")
        database = OgnIsaacCreateRTXLidarScanBufferDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:accuracyErrorAzimuthDeg"))
        attribute = test_node.get_attribute("inputs:accuracyErrorAzimuthDeg")
        db_value = database.inputs.accuracyErrorAzimuthDeg
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:accuracyErrorElevationDeg"))
        attribute = test_node.get_attribute("inputs:accuracyErrorElevationDeg")
        db_value = database.inputs.accuracyErrorElevationDeg
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:accuracyErrorPosition"))
        attribute = test_node.get_attribute("inputs:accuracyErrorPosition")
        db_value = database.inputs.accuracyErrorPosition
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cudaDeviceIndex"))
        attribute = test_node.get_attribute("inputs:cudaDeviceIndex")
        db_value = database.inputs.cudaDeviceIndex
        expected_value = -1
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

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
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputAzimuth"))
        attribute = test_node.get_attribute("inputs:outputAzimuth")
        db_value = database.inputs.outputAzimuth
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputBeamId"))
        attribute = test_node.get_attribute("inputs:outputBeamId")
        db_value = database.inputs.outputBeamId
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputDistance"))
        attribute = test_node.get_attribute("inputs:outputDistance")
        db_value = database.inputs.outputDistance
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputElevation"))
        attribute = test_node.get_attribute("inputs:outputElevation")
        db_value = database.inputs.outputElevation
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputEmitterId"))
        attribute = test_node.get_attribute("inputs:outputEmitterId")
        db_value = database.inputs.outputEmitterId
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputIntensity"))
        attribute = test_node.get_attribute("inputs:outputIntensity")
        db_value = database.inputs.outputIntensity
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputMaterialId"))
        attribute = test_node.get_attribute("inputs:outputMaterialId")
        db_value = database.inputs.outputMaterialId
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputNormal"))
        attribute = test_node.get_attribute("inputs:outputNormal")
        db_value = database.inputs.outputNormal
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputObjectId"))
        attribute = test_node.get_attribute("inputs:outputObjectId")
        db_value = database.inputs.outputObjectId
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputTimestamp"))
        attribute = test_node.get_attribute("inputs:outputTimestamp")
        db_value = database.inputs.outputTimestamp
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:outputVelocity"))
        attribute = test_node.get_attribute("inputs:outputVelocity")
        db_value = database.inputs.outputVelocity
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

        self.assertTrue(test_node.get_attribute_exists("inputs:transformPoints"))
        attribute = test_node.get_attribute("inputs:transformPoints")
        db_value = database.inputs.transformPoints
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuthBufferSize"))
        attribute = test_node.get_attribute("outputs:azimuthBufferSize")
        db_value = database.outputs.azimuthBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuthDataType"))
        attribute = test_node.get_attribute("outputs:azimuthDataType")
        db_value = database.outputs.azimuthDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuthPtr"))
        attribute = test_node.get_attribute("outputs:azimuthPtr")
        db_value = database.outputs.azimuthPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:beamIdBufferSize"))
        attribute = test_node.get_attribute("outputs:beamIdBufferSize")
        db_value = database.outputs.beamIdBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:beamIdDataType"))
        attribute = test_node.get_attribute("outputs:beamIdDataType")
        db_value = database.outputs.beamIdDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:beamIdPtr"))
        attribute = test_node.get_attribute("outputs:beamIdPtr")
        db_value = database.outputs.beamIdPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:bufferSize"))
        attribute = test_node.get_attribute("outputs:bufferSize")
        db_value = database.outputs.bufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:cudaDeviceIndex"))
        attribute = test_node.get_attribute("outputs:cudaDeviceIndex")
        db_value = database.outputs.cudaDeviceIndex

        self.assertTrue(test_node.get_attribute_exists("outputs:dataPtr"))
        attribute = test_node.get_attribute("outputs:dataPtr")
        db_value = database.outputs.dataPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:distanceBufferSize"))
        attribute = test_node.get_attribute("outputs:distanceBufferSize")
        db_value = database.outputs.distanceBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:distanceDataType"))
        attribute = test_node.get_attribute("outputs:distanceDataType")
        db_value = database.outputs.distanceDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:distancePtr"))
        attribute = test_node.get_attribute("outputs:distancePtr")
        db_value = database.outputs.distancePtr

        self.assertTrue(test_node.get_attribute_exists("outputs:elevationBufferSize"))
        attribute = test_node.get_attribute("outputs:elevationBufferSize")
        db_value = database.outputs.elevationBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:elevationDataType"))
        attribute = test_node.get_attribute("outputs:elevationDataType")
        db_value = database.outputs.elevationDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:elevationPtr"))
        attribute = test_node.get_attribute("outputs:elevationPtr")
        db_value = database.outputs.elevationPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:emitterIdBufferSize"))
        attribute = test_node.get_attribute("outputs:emitterIdBufferSize")
        db_value = database.outputs.emitterIdBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:emitterIdDataType"))
        attribute = test_node.get_attribute("outputs:emitterIdDataType")
        db_value = database.outputs.emitterIdDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:emitterIdPtr"))
        attribute = test_node.get_attribute("outputs:emitterIdPtr")
        db_value = database.outputs.emitterIdPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        db_value = database.outputs.exec

        self.assertTrue(test_node.get_attribute_exists("outputs:height"))
        attribute = test_node.get_attribute("outputs:height")
        db_value = database.outputs.height

        self.assertTrue(test_node.get_attribute_exists("outputs:indexBufferSize"))
        attribute = test_node.get_attribute("outputs:indexBufferSize")
        db_value = database.outputs.indexBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:indexDataType"))
        attribute = test_node.get_attribute("outputs:indexDataType")
        db_value = database.outputs.indexDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:indexPtr"))
        attribute = test_node.get_attribute("outputs:indexPtr")
        db_value = database.outputs.indexPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:intensityBufferSize"))
        attribute = test_node.get_attribute("outputs:intensityBufferSize")
        db_value = database.outputs.intensityBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:intensityDataType"))
        attribute = test_node.get_attribute("outputs:intensityDataType")
        db_value = database.outputs.intensityDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:intensityPtr"))
        attribute = test_node.get_attribute("outputs:intensityPtr")
        db_value = database.outputs.intensityPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:materialIdBufferSize"))
        attribute = test_node.get_attribute("outputs:materialIdBufferSize")
        db_value = database.outputs.materialIdBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:materialIdDataType"))
        attribute = test_node.get_attribute("outputs:materialIdDataType")
        db_value = database.outputs.materialIdDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:materialIdPtr"))
        attribute = test_node.get_attribute("outputs:materialIdPtr")
        db_value = database.outputs.materialIdPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:normalBufferSize"))
        attribute = test_node.get_attribute("outputs:normalBufferSize")
        db_value = database.outputs.normalBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:normalDataType"))
        attribute = test_node.get_attribute("outputs:normalDataType")
        db_value = database.outputs.normalDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:normalPtr"))
        attribute = test_node.get_attribute("outputs:normalPtr")
        db_value = database.outputs.normalPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:numChannels"))
        attribute = test_node.get_attribute("outputs:numChannels")
        db_value = database.outputs.numChannels

        self.assertTrue(test_node.get_attribute_exists("outputs:numEchos"))
        attribute = test_node.get_attribute("outputs:numEchos")
        db_value = database.outputs.numEchos

        self.assertTrue(test_node.get_attribute_exists("outputs:numReturnsPerScan"))
        attribute = test_node.get_attribute("outputs:numReturnsPerScan")
        db_value = database.outputs.numReturnsPerScan

        self.assertTrue(test_node.get_attribute_exists("outputs:objectIdBufferSize"))
        attribute = test_node.get_attribute("outputs:objectIdBufferSize")
        db_value = database.outputs.objectIdBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:objectIdDataType"))
        attribute = test_node.get_attribute("outputs:objectIdDataType")
        db_value = database.outputs.objectIdDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:objectIdPtr"))
        attribute = test_node.get_attribute("outputs:objectIdPtr")
        db_value = database.outputs.objectIdPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:renderProductPath"))
        attribute = test_node.get_attribute("outputs:renderProductPath")
        db_value = database.outputs.renderProductPath

        self.assertTrue(test_node.get_attribute_exists("outputs:ticksPerScan"))
        attribute = test_node.get_attribute("outputs:ticksPerScan")
        db_value = database.outputs.ticksPerScan

        self.assertTrue(test_node.get_attribute_exists("outputs:timestampBufferSize"))
        attribute = test_node.get_attribute("outputs:timestampBufferSize")
        db_value = database.outputs.timestampBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:timestampDataType"))
        attribute = test_node.get_attribute("outputs:timestampDataType")
        db_value = database.outputs.timestampDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:timestampPtr"))
        attribute = test_node.get_attribute("outputs:timestampPtr")
        db_value = database.outputs.timestampPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:transform"))
        attribute = test_node.get_attribute("outputs:transform")
        db_value = database.outputs.transform

        self.assertTrue(test_node.get_attribute_exists("outputs:velocityBufferSize"))
        attribute = test_node.get_attribute("outputs:velocityBufferSize")
        db_value = database.outputs.velocityBufferSize

        self.assertTrue(test_node.get_attribute_exists("outputs:velocityDataType"))
        attribute = test_node.get_attribute("outputs:velocityDataType")
        db_value = database.outputs.velocityDataType

        self.assertTrue(test_node.get_attribute_exists("outputs:velocityPtr"))
        attribute = test_node.get_attribute("outputs:velocityPtr")
        db_value = database.outputs.velocityPtr

        self.assertTrue(test_node.get_attribute_exists("outputs:width"))
        attribute = test_node.get_attribute("outputs:width")
        db_value = database.outputs.width
