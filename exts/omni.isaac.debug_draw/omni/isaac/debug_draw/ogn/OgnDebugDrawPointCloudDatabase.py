"""Support for simplified access to data on nodes of type omni.isaac.debug_draw.DebugDrawPointCloud

Take a point cloud as input and display it in the scene.
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnDebugDrawPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.debug_draw.DebugDrawPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.bufferSize
            inputs.color
            inputs.dataPtr
            inputs.doTransform
            inputs.exec
            inputs.size
            inputs.testMode
            inputs.transform
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
        ('inputs:bufferSize', 'uint64', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, 0, False, ''),
        ('inputs:color', 'color4f', 0, None, 'Color of points', {ogn.MetadataKeys.DEFAULT: '[0.75, 0.75, 1, 1]'}, True, [0.75, 0.75, 1, 1], False, ''),
        ('inputs:dataPtr', 'uint64', 0, None, 'Buffer of points containing point cloud data', {}, True, 0, False, ''),
        ('inputs:doTransform', 'bool', 0, 'Do Transform', 'Translate and Rotate point cloud by transform', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:exec', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:size', 'float', 0, None, 'Size of points', {ogn.MetadataKeys.DEFAULT: '0.02'}, True, 0.02, False, ''),
        ('inputs:testMode', 'bool', 0, None, 'Act as Writer with no rendering', {}, True, False, False, ''),
        ('inputs:transform', 'matrix4d', 0, None, 'The matrix to transform the points by', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.color = og.AttributeRole.COLOR
        role_data.inputs.exec = og.AttributeRole.EXECUTION
        role_data.inputs.transform = og.AttributeRole.MATRIX
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
        def color(self):
            data_view = og.AttributeValueHelper(self._attributes.color)
            return data_view.get()

        @color.setter
        def color(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.color)
            data_view = og.AttributeValueHelper(self._attributes.color)
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
        def doTransform(self):
            data_view = og.AttributeValueHelper(self._attributes.doTransform)
            return data_view.get()

        @doTransform.setter
        def doTransform(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.doTransform)
            data_view = og.AttributeValueHelper(self._attributes.doTransform)
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
        def size(self):
            data_view = og.AttributeValueHelper(self._attributes.size)
            return data_view.get()

        @size.setter
        def size(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.size)
            data_view = og.AttributeValueHelper(self._attributes.size)
            data_view.set(value)

        @property
        def testMode(self):
            data_view = og.AttributeValueHelper(self._attributes.testMode)
            return data_view.get()

        @testMode.setter
        def testMode(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.testMode)
            data_view = og.AttributeValueHelper(self._attributes.testMode)
            data_view.set(value)

        @property
        def transform(self):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            return data_view.get()

        @transform.setter
        def transform(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.transform)
            data_view = og.AttributeValueHelper(self._attributes.transform)
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
        self.inputs = OgnDebugDrawPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnDebugDrawPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnDebugDrawPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
