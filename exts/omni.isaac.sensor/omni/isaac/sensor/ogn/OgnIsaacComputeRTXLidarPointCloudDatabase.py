"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarPointCloud

This node reads from the an RTX Lidar sensor and holds point cloud data buffers
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacComputeRTXLidarPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.accuracyErrorAzimuthDeg
            inputs.accuracyErrorElevationDeg
            inputs.accuracyErrorPosition
            inputs.dataPtr
            inputs.exec
            inputs.keepOnlyPositiveDistance
            inputs.renderProductPath
        Outputs:
            outputs.azimuth
            outputs.bufferSize
            outputs.cudaDeviceIndex
            outputs.dataPtr
            outputs.elevation
            outputs.exec
            outputs.height
            outputs.intensity
            outputs.range
            outputs.transform
            outputs.width
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
        ('inputs:accuracyErrorAzimuthDeg', 'float', 0, 'Error Azimuth', 'Accuracy error of azimuth in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorElevationDeg', 'float', 0, 'Error Elevation', 'Accuracy error of elevation in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorPosition', 'float3', 0, 'Error Position', 'Position offset applied to all points equally', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:dataPtr', 'uint64', 0, 'LiDAR render result', 'Pointer to LiDAR render result', {}, True, 0, False, ''),
        ('inputs:exec', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:keepOnlyPositiveDistance', 'bool', 0, 'Keep Only Positive Distance', 'Keep points only if the return distance is > 0', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the renderProduct to wait for being rendered', {}, True, "", False, ''),
        ('outputs:azimuth', 'float[]', 0, None, 'azimuth in rad [-pi,pi]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:bufferSize', 'uint64', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('outputs:dataPtr', 'uint64', 0, 'Point Cloud Data', 'Buffer of points containing point cloud data in Lidar coordinates', {}, True, None, False, ''),
        ('outputs:elevation', 'float[]', 0, None, 'elevation in rad [-pi/2, pi/2]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:exec', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height of point cloud buffer, will always return 1', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('outputs:intensity', 'float[]', 0, None, 'intensity [0,1]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:range', 'float[]', 0, None, 'range in m', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:transform', 'matrix4d', 0, None, 'The transform matrix from lidar to world coordinates', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, '3 x Width or number of points in point cloud buffer', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
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
        def accuracyErrorAzimuthDeg(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorAzimuthDeg)
            return data_view.get()

        @accuracyErrorAzimuthDeg.setter
        def accuracyErrorAzimuthDeg(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorAzimuthDeg)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorAzimuthDeg)
            data_view.set(value)

        @property
        def accuracyErrorElevationDeg(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorElevationDeg)
            return data_view.get()

        @accuracyErrorElevationDeg.setter
        def accuracyErrorElevationDeg(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorElevationDeg)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorElevationDeg)
            data_view.set(value)

        @property
        def accuracyErrorPosition(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorPosition)
            return data_view.get()

        @accuracyErrorPosition.setter
        def accuracyErrorPosition(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorPosition)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorPosition)
            data_view.set(value)

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
        def keepOnlyPositiveDistance(self):
            data_view = og.AttributeValueHelper(self._attributes.keepOnlyPositiveDistance)
            return data_view.get()

        @keepOnlyPositiveDistance.setter
        def keepOnlyPositiveDistance(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.keepOnlyPositiveDistance)
            data_view = og.AttributeValueHelper(self._attributes.keepOnlyPositiveDistance)
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
            self.intensity_size = 0
            self.range_size = 0
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
        def height(self):
            data_view = og.AttributeValueHelper(self._attributes.height)
            return data_view.get()

        @height.setter
        def height(self, value):
            data_view = og.AttributeValueHelper(self._attributes.height)
            data_view.set(value)

        @property
        def intensity(self):
            data_view = og.AttributeValueHelper(self._attributes.intensity)
            return data_view.get(reserved_element_count=self.intensity_size)

        @intensity.setter
        def intensity(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensity)
            data_view.set(value)
            self.intensity_size = data_view.get_array_size()

        @property
        def range(self):
            data_view = og.AttributeValueHelper(self._attributes.range)
            return data_view.get(reserved_element_count=self.range_size)

        @range.setter
        def range(self, value):
            data_view = og.AttributeValueHelper(self._attributes.range)
            data_view.set(value)
            self.range_size = data_view.get_array_size()

        @property
        def transform(self):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            return data_view.get()

        @transform.setter
        def transform(self, value):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            data_view.set(value)

        @property
        def width(self):
            data_view = og.AttributeValueHelper(self._attributes.width)
            return data_view.get()

        @width.setter
        def width(self, value):
            data_view = og.AttributeValueHelper(self._attributes.width)
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
        self.inputs = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
