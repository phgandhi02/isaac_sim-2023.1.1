"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacConvertDepthToPointCloud

Converts a 32FC1 image buffer into Point Cloud data
"""

import carb

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacConvertDepthToPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacConvertDepthToPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.bufferSize
            inputs.cudaDeviceIndex
            inputs.dataPtr
            inputs.execIn
            inputs.focalLength
            inputs.format
            inputs.height
            inputs.horizontalAperture
            inputs.verticalAperture
            inputs.width
        Outputs:
            outputs.bufferSize
            outputs.cudaDeviceIndex
            outputs.dataPtr
            outputs.execOut
            outputs.height
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
        ('inputs:bufferSize', 'uint', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, 0, False, ''),
        ('inputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('inputs:dataPtr', 'uint64', 0, None, 'Pointer to the raw rgba array data', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:focalLength', 'float', 0, None, 'focal length', {}, True, 0.0, False, ''),
        ('inputs:format', 'uint64', 0, None, 'Format', {ogn.MetadataKeys.DEFAULT: '33'}, True, 33, False, ''),
        ('inputs:height', 'uint', 0, None, 'Buffer array height, same as input', {}, True, 0, False, ''),
        ('inputs:horizontalAperture', 'float', 0, None, 'horizontal aperture', {}, True, 0.0, False, ''),
        ('inputs:verticalAperture', 'float', 0, None, 'vertical aperture', {}, True, 0.0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Buffer array width, same as input', {}, True, 0, False, ''),
        ('outputs:bufferSize', 'uint', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('outputs:dataPtr', 'uint64', 0, None, 'Pointer to the rgb buffer data', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when lidar sensor has completed a full scan', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height of point cloud buffer, will always return 1', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('outputs:width', 'uint', 0, None, 'number of points in point cloud buffer', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
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
        def bufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            return data_view.get()

        @bufferSize.setter
        def bufferSize(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.bufferSize)
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            data_view.set(value)

        @property
        def cudaDeviceIndex(self):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            return data_view.get()

        @cudaDeviceIndex.setter
        def cudaDeviceIndex(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cudaDeviceIndex)
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
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
        def execIn(self):
            data_view = og.AttributeValueHelper(self._attributes.execIn)
            return data_view.get()

        @execIn.setter
        def execIn(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.execIn)
            data_view = og.AttributeValueHelper(self._attributes.execIn)
            data_view.set(value)

        @property
        def focalLength(self):
            data_view = og.AttributeValueHelper(self._attributes.focalLength)
            return data_view.get()

        @focalLength.setter
        def focalLength(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.focalLength)
            data_view = og.AttributeValueHelper(self._attributes.focalLength)
            data_view.set(value)

        @property
        def format(self):
            data_view = og.AttributeValueHelper(self._attributes.format)
            return data_view.get()

        @format.setter
        def format(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.format)
            data_view = og.AttributeValueHelper(self._attributes.format)
            data_view.set(value)

        @property
        def height(self):
            data_view = og.AttributeValueHelper(self._attributes.height)
            return data_view.get()

        @height.setter
        def height(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.height)
            data_view = og.AttributeValueHelper(self._attributes.height)
            data_view.set(value)

        @property
        def horizontalAperture(self):
            data_view = og.AttributeValueHelper(self._attributes.horizontalAperture)
            return data_view.get()

        @horizontalAperture.setter
        def horizontalAperture(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.horizontalAperture)
            data_view = og.AttributeValueHelper(self._attributes.horizontalAperture)
            data_view.set(value)

        @property
        def verticalAperture(self):
            data_view = og.AttributeValueHelper(self._attributes.verticalAperture)
            return data_view.get()

        @verticalAperture.setter
        def verticalAperture(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.verticalAperture)
            data_view = og.AttributeValueHelper(self._attributes.verticalAperture)
            data_view.set(value)

        @property
        def width(self):
            data_view = og.AttributeValueHelper(self._attributes.width)
            return data_view.get()

        @width.setter
        def width(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.width)
            data_view = og.AttributeValueHelper(self._attributes.width)
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
            self._batchedWriteValues = { }

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
        def execOut(self):
            data_view = og.AttributeValueHelper(self._attributes.execOut)
            return data_view.get()

        @execOut.setter
        def execOut(self, value):
            data_view = og.AttributeValueHelper(self._attributes.execOut)
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
        self.inputs = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
