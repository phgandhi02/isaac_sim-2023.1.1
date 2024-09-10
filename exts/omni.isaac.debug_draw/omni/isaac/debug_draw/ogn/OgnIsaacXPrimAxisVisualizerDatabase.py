"""Support for simplified access to data on nodes of type omni.isaac.debug_draw.IsaacXPrimAxisVisualizer

displays the x,y,z axis of an xPrim for visualization.
"""

import carb
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacXPrimAxisVisualizerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.debug_draw.IsaacXPrimAxisVisualizer

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.length
            inputs.thickness
            inputs.xPrim
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
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:length', 'float', 0, None, 'Length of the axis lines', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('inputs:thickness', 'float', 0, None, 'Thickness of the axis lines', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('inputs:xPrim', 'target', 0, 'xPrim', 'Usd prim to visualize', {}, True, [], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.xPrim = og.AttributeRole.TARGET
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
        def length(self):
            data_view = og.AttributeValueHelper(self._attributes.length)
            return data_view.get()

        @length.setter
        def length(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.length)
            data_view = og.AttributeValueHelper(self._attributes.length)
            data_view.set(value)

        @property
        def thickness(self):
            data_view = og.AttributeValueHelper(self._attributes.thickness)
            return data_view.get()

        @thickness.setter
        def thickness(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.thickness)
            data_view = og.AttributeValueHelper(self._attributes.thickness)
            data_view.set(value)

        @property
        def xPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.xPrim)
            return data_view.get()

        @xPrim.setter
        def xPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.xPrim)
            data_view = og.AttributeValueHelper(self._attributes.xPrim)
            data_view.set(value)
            self.xPrim_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacXPrimAxisVisualizerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacXPrimAxisVisualizerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacXPrimAxisVisualizerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
