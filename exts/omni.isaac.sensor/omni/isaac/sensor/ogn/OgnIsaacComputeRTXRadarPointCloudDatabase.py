"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXRadarPointCloud

This node reads from the an RTX Radar sensor and holds point cloud data buffers
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacComputeRTXRadarPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXRadarPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.dataPtr
            inputs.exec
            inputs.renderProductPath
        Outputs:
            outputs.azimuth
            outputs.bufferSize
            outputs.cudaDeviceIndex
            outputs.cycleCnt
            outputs.dataPtr
            outputs.elevation
            outputs.exec
            outputs.materialId
            outputs.maxAzRad
            outputs.maxElRad
            outputs.maxRangeM
            outputs.maxVelMps
            outputs.minAzRad
            outputs.minElRad
            outputs.minVelMps
            outputs.numDetections
            outputs.objectId
            outputs.radialDistance
            outputs.radialVelocity
            outputs.rcs
            outputs.scanIdx
            outputs.semanticId
            outputs.sensorID
            outputs.syncData
            outputs.timeStampNS
            outputs.transform
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 41, 3)
    TARGET_VERSION = (2, 139, 12)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:dataPtr', 'uint64', 0, 'Data Pointer', 'Pointer to Radar render result.', {}, True, 0, False, ''),
        ('inputs:exec', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the renderProduct to wait for being rendered', {}, True, "", False, ''),
        ('outputs:azimuth', 'float[]', 0, None, 'Azimuth angle (radians)', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:bufferSize', 'uint64', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('outputs:cycleCnt', 'uint64', 0, None, 'Scan cycle count', {}, True, None, False, ''),
        ('outputs:dataPtr', 'uint64', 0, 'Point Cloud Data', 'Buffer of 3d points containing point cloud data in Radar coordinates', {}, True, None, False, ''),
        ('outputs:elevation', 'float[]', 0, None, 'Angle of elevation (radians)', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:exec', 'execution', 0, None, 'Output execution triggers when Radar sensor has data', {}, True, None, False, ''),
        ('outputs:materialId', 'uint[]', 0, None, 'material ID', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:maxAzRad', 'float', 0, None, 'The max unambiguous azimuth for the scan', {}, True, None, False, ''),
        ('outputs:maxElRad', 'float', 0, None, 'The max unambiguous elevation for the scan', {}, True, None, False, ''),
        ('outputs:maxRangeM', 'float', 0, None, 'The max unambiguous range for the scan', {}, True, None, False, ''),
        ('outputs:maxVelMps', 'float', 0, None, 'The max unambiguous velocity for the scan', {}, True, None, False, ''),
        ('outputs:minAzRad', 'float', 0, None, 'The min unambiguous azimuth for the scan', {}, True, None, False, ''),
        ('outputs:minElRad', 'float', 0, None, 'The min unambiguous elevation for the scan', {}, True, None, False, ''),
        ('outputs:minVelMps', 'float', 0, None, 'The min unambiguous velocity for the scan', {}, True, None, False, ''),
        ('outputs:numDetections', 'uint', 0, None, 'The number of valid detections in the array', {}, True, None, False, ''),
        ('outputs:objectId', 'uint[]', 0, None, 'object ID', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:radialDistance', 'float[]', 0, None, 'Radial distance (m)', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:radialVelocity', 'float[]', 0, None, 'Radial velocity (m/s)', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:rcs', 'float[]', 0, None, 'Radar cross section in decibels referenced to a square meter (dBsm)', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:scanIdx', 'uchar', 0, None, 'Scan index for sensors with multi scan support', {}, True, None, False, ''),
        ('outputs:semanticId', 'uint[]', 0, None, 'semantic ID', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:sensorID', 'uchar', 0, None, 'Sensor Id for sensor that generated the scan', {}, True, None, False, ''),
        ('outputs:syncData', 'uint64', 0, None, 'Pointer to SyncData Sync primitives for syncing with model', {}, True, None, False, ''),
        ('outputs:timeStampNS', 'uint64', 0, None, 'Scan timestamp in nanoseconds', {}, True, None, False, ''),
        ('outputs:transform', 'matrix4d', 0, None, 'The input matrix transformed from Radar to World', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.transform = og.AttributeRole.MATRIX
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = []
            self._batchedReadValues = []

        @property
        def dataPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            return data_view.get()

        @dataPtr.setter
        def dataPtr(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.dataPtr)
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            data_view.set(value)

        @property
        def exec(self):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            return data_view.get()

        @exec.setter
        def exec(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.exec)
            data_view = og.AttributeValueHelper(self._attributes.exec)
            data_view.set(value)

        @property
        def renderProductPath(self):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
            return data_view.get()

        @renderProductPath.setter
        def renderProductPath(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.renderProductPath)
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
            data_view.set(value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.azimuth_size = 0
            self.elevation_size = 0
            self.materialId_size = 0
            self.objectId_size = 0
            self.radialDistance_size = 0
            self.radialVelocity_size = 0
            self.rcs_size = 0
            self.semanticId_size = 0
            self._batchedWriteValues = { }

        @property
        def azimuth(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuth)
            return data_view.get(reserved_element_count=self.azimuth_size)

        @azimuth.setter
        def azimuth(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuth)
            data_view.set(value)
            self.azimuth_size = data_view.get_array_size()

        @property
        def bufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            return data_view.get()

        @bufferSize.setter
        def bufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            data_view.set(value)

        @property
        def cudaDeviceIndex(self):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            return data_view.get()

        @cudaDeviceIndex.setter
        def cudaDeviceIndex(self, value):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            data_view.set(value)

        @property
        def cycleCnt(self):
            data_view = og.AttributeValueHelper(self._attributes.cycleCnt)
            return data_view.get()

        @cycleCnt.setter
        def cycleCnt(self, value):
            data_view = og.AttributeValueHelper(self._attributes.cycleCnt)
            data_view.set(value)

        @property
        def dataPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            return data_view.get()

        @dataPtr.setter
        def dataPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            data_view.set(value)

        @property
        def elevation(self):
            data_view = og.AttributeValueHelper(self._attributes.elevation)
            return data_view.get(reserved_element_count=self.elevation_size)

        @elevation.setter
        def elevation(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevation)
            data_view.set(value)
            self.elevation_size = data_view.get_array_size()

        @property
        def exec(self):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            return data_view.get()

        @exec.setter
        def exec(self, value):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            data_view.set(value)

        @property
        def materialId(self):
            data_view = og.AttributeValueHelper(self._attributes.materialId)
            return data_view.get(reserved_element_count=self.materialId_size)

        @materialId.setter
        def materialId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialId)
            data_view.set(value)
            self.materialId_size = data_view.get_array_size()

        @property
        def maxAzRad(self):
            data_view = og.AttributeValueHelper(self._attributes.maxAzRad)
            return data_view.get()

        @maxAzRad.setter
        def maxAzRad(self, value):
            data_view = og.AttributeValueHelper(self._attributes.maxAzRad)
            data_view.set(value)

        @property
        def maxElRad(self):
            data_view = og.AttributeValueHelper(self._attributes.maxElRad)
            return data_view.get()

        @maxElRad.setter
        def maxElRad(self, value):
            data_view = og.AttributeValueHelper(self._attributes.maxElRad)
            data_view.set(value)

        @property
        def maxRangeM(self):
            data_view = og.AttributeValueHelper(self._attributes.maxRangeM)
            return data_view.get()

        @maxRangeM.setter
        def maxRangeM(self, value):
            data_view = og.AttributeValueHelper(self._attributes.maxRangeM)
            data_view.set(value)

        @property
        def maxVelMps(self):
            data_view = og.AttributeValueHelper(self._attributes.maxVelMps)
            return data_view.get()

        @maxVelMps.setter
        def maxVelMps(self, value):
            data_view = og.AttributeValueHelper(self._attributes.maxVelMps)
            data_view.set(value)

        @property
        def minAzRad(self):
            data_view = og.AttributeValueHelper(self._attributes.minAzRad)
            return data_view.get()

        @minAzRad.setter
        def minAzRad(self, value):
            data_view = og.AttributeValueHelper(self._attributes.minAzRad)
            data_view.set(value)

        @property
        def minElRad(self):
            data_view = og.AttributeValueHelper(self._attributes.minElRad)
            return data_view.get()

        @minElRad.setter
        def minElRad(self, value):
            data_view = og.AttributeValueHelper(self._attributes.minElRad)
            data_view.set(value)

        @property
        def minVelMps(self):
            data_view = og.AttributeValueHelper(self._attributes.minVelMps)
            return data_view.get()

        @minVelMps.setter
        def minVelMps(self, value):
            data_view = og.AttributeValueHelper(self._attributes.minVelMps)
            data_view.set(value)

        @property
        def numDetections(self):
            data_view = og.AttributeValueHelper(self._attributes.numDetections)
            return data_view.get()

        @numDetections.setter
        def numDetections(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numDetections)
            data_view.set(value)

        @property
        def objectId(self):
            data_view = og.AttributeValueHelper(self._attributes.objectId)
            return data_view.get(reserved_element_count=self.objectId_size)

        @objectId.setter
        def objectId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectId)
            data_view.set(value)
            self.objectId_size = data_view.get_array_size()

        @property
        def radialDistance(self):
            data_view = og.AttributeValueHelper(self._attributes.radialDistance)
            return data_view.get(reserved_element_count=self.radialDistance_size)

        @radialDistance.setter
        def radialDistance(self, value):
            data_view = og.AttributeValueHelper(self._attributes.radialDistance)
            data_view.set(value)
            self.radialDistance_size = data_view.get_array_size()

        @property
        def radialVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.radialVelocity)
            return data_view.get(reserved_element_count=self.radialVelocity_size)

        @radialVelocity.setter
        def radialVelocity(self, value):
            data_view = og.AttributeValueHelper(self._attributes.radialVelocity)
            data_view.set(value)
            self.radialVelocity_size = data_view.get_array_size()

        @property
        def rcs(self):
            data_view = og.AttributeValueHelper(self._attributes.rcs)
            return data_view.get(reserved_element_count=self.rcs_size)

        @rcs.setter
        def rcs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.rcs)
            data_view.set(value)
            self.rcs_size = data_view.get_array_size()

        @property
        def scanIdx(self):
            data_view = og.AttributeValueHelper(self._attributes.scanIdx)
            return data_view.get()

        @scanIdx.setter
        def scanIdx(self, value):
            data_view = og.AttributeValueHelper(self._attributes.scanIdx)
            data_view.set(value)

        @property
        def semanticId(self):
            data_view = og.AttributeValueHelper(self._attributes.semanticId)
            return data_view.get(reserved_element_count=self.semanticId_size)

        @semanticId.setter
        def semanticId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.semanticId)
            data_view.set(value)
            self.semanticId_size = data_view.get_array_size()

        @property
        def sensorID(self):
            data_view = og.AttributeValueHelper(self._attributes.sensorID)
            return data_view.get()

        @sensorID.setter
        def sensorID(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sensorID)
            data_view.set(value)

        @property
        def syncData(self):
            data_view = og.AttributeValueHelper(self._attributes.syncData)
            return data_view.get()

        @syncData.setter
        def syncData(self, value):
            data_view = og.AttributeValueHelper(self._attributes.syncData)
            data_view.set(value)

        @property
        def timeStampNS(self):
            data_view = og.AttributeValueHelper(self._attributes.timeStampNS)
            return data_view.get()

        @timeStampNS.setter
        def timeStampNS(self, value):
            data_view = og.AttributeValueHelper(self._attributes.timeStampNS)
            data_view.set(value)

        @property
        def transform(self):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            return data_view.get()

        @transform.setter
        def transform(self, value):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            data_view.set(value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnIsaacComputeRTXRadarPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacComputeRTXRadarPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacComputeRTXRadarPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
