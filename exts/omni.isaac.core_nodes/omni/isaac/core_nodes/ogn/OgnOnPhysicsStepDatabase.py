"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.OnPhysicsStep

Executes an output execution pulse for every physics Simulation Step
"""


import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnOnPhysicsStepDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.OnPhysicsStep

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Outputs:
            outputs.deltaSimulationTime
            outputs.deltaSystemTime
            outputs.step
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
        ('outputs:deltaSimulationTime', 'double', 0, 'Simulation Delta Time', 'Simulation Time elapsed since the last update (seconds)', {}, True, None, False, ''),
        ('outputs:deltaSystemTime', 'double', 0, 'System Delta Time', 'System Time elapsed since last update (seconds)', {}, True, None, False, ''),
        ('outputs:step', 'execution', 0, 'Step', 'The execution output', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.outputs.step = og.AttributeRole.EXECUTION
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
        def deltaSimulationTime(self):
            data_view = og.AttributeValueHelper(self._attributes.deltaSimulationTime)
            return data_view.get()

        @deltaSimulationTime.setter
        def deltaSimulationTime(self, value):
            data_view = og.AttributeValueHelper(self._attributes.deltaSimulationTime)
            data_view.set(value)

        @property
        def deltaSystemTime(self):
            data_view = og.AttributeValueHelper(self._attributes.deltaSystemTime)
            return data_view.get()

        @deltaSystemTime.setter
        def deltaSystemTime(self, value):
            data_view = og.AttributeValueHelper(self._attributes.deltaSystemTime)
            data_view.set(value)

        @property
        def step(self):
            data_view = og.AttributeValueHelper(self._attributes.step)
            return data_view.get()

        @step.setter
        def step(self, value):
            data_view = og.AttributeValueHelper(self._attributes.step)
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
        self.inputs = OgnOnPhysicsStepDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnOnPhysicsStepDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnOnPhysicsStepDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
