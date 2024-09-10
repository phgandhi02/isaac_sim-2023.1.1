"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2Context

This node creates a ROS2 Context for a given domain ID
"""

import carb

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2ContextDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2Context

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.domain_id
            inputs.useDomainIDEnvVar
        Outputs:
            outputs.context
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
        ('inputs:domain_id', 'uchar', 0, None, 'Domain ID for ROS context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:useDomainIDEnvVar', 'bool', 0, None, 'Set to true to use ROS_DOMAIN_ID environment variable if set. Defaults to domain_id if not found', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('outputs:context', 'uint64', 0, None, 'handle to initialized ROS2 context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
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
        def domain_id(self):
            data_view = og.AttributeValueHelper(self._attributes.domain_id)
            return data_view.get()

        @domain_id.setter
        def domain_id(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.domain_id)
            data_view = og.AttributeValueHelper(self._attributes.domain_id)
            data_view.set(value)

        @property
        def useDomainIDEnvVar(self):
            data_view = og.AttributeValueHelper(self._attributes.useDomainIDEnvVar)
            return data_view.get()

        @useDomainIDEnvVar.setter
        def useDomainIDEnvVar(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.useDomainIDEnvVar)
            data_view = og.AttributeValueHelper(self._attributes.useDomainIDEnvVar)
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
        def context(self):
            data_view = og.AttributeValueHelper(self._attributes.context)
            return data_view.get()

        @context.setter
        def context(self, value):
            data_view = og.AttributeValueHelper(self._attributes.context)
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
        self.inputs = OgnROS2ContextDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2ContextDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2ContextDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
