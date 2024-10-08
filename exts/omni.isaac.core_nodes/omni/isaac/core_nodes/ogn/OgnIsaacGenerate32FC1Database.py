"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacGenerate32FC1

Isaac Sim Node that generates a constant 32FC1 buffer
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacGenerate32FC1Database(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacGenerate32FC1

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.height
            inputs.value
            inputs.width
        Outputs:
            outputs.data
            outputs.encoding
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
        ('inputs:height', 'uint', 0, None, 'Height for output image', {ogn.MetadataKeys.DEFAULT: '100'}, True, 100, False, ''),
        ('inputs:value', 'float', 0, None, 'Value for output image', {}, True, 0.0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width for output image', {ogn.MetadataKeys.DEFAULT: '100'}, True, 100, False, ''),
        ('outputs:data', 'uchar[]', 0, None, 'Buffer 32FC1 array data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:encoding', 'token', 0, None, 'Encoding as a token', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height for output image', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Width for output image', {}, True, None, False, ''),
    ])

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
        def value(self):
            data_view = og.AttributeValueHelper(self._attributes.value)
            return data_view.get()

        @value.setter
        def value(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.value)
            data_view = og.AttributeValueHelper(self._attributes.value)
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
            self.data_size = 0
            self._batchedWriteValues = { }

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get(reserved_element_count=self.data_size)

        @data.setter
        def data(self, value):
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value)
            self.data_size = data_view.get_array_size()

        @property
        def encoding(self):
            data_view = og.AttributeValueHelper(self._attributes.encoding)
            return data_view.get()

        @encoding.setter
        def encoding(self, value):
            data_view = og.AttributeValueHelper(self._attributes.encoding)
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
        self.inputs = OgnIsaacGenerate32FC1Database.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacGenerate32FC1Database.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacGenerate32FC1Database.ValuesForState(node, self.attributes.state, dynamic_attributes)
