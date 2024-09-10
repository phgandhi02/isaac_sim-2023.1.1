"""Support for simplified access to data on nodes of type omni.isaac.conveyor.IsaacConveyor

Conveyor Belt
"""

import numpy
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacConveyorDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.conveyor.IsaacConveyor

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.animateDirection
            inputs.animateScale
            inputs.animateTexture
            inputs.conveyorPrim
            inputs.curved
            inputs.delta
            inputs.direction
            inputs.enabled
            inputs.onStep
            inputs.velocity
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
        ('inputs:animateDirection', 'float2', 0, None, 'relative direction to apply to UV texture', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0]'}, True, [1.0, 0.0], False, ''),
        ('inputs:animateScale', 'float', 0, None, 'how fast to scale animate texture', {ogn.MetadataKeys.DEFAULT: '1.0'}, True, 1.0, False, ''),
        ('inputs:animateTexture', 'bool', 0, None, 'If configured, Animates the texture based on the conveyor velocity. may affect performance specially if multiple instances are added to a scene.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:conveyorPrim', 'target', 0, None, 'the prim reference to the conveyor', {}, True, [], False, ''),
        ('inputs:curved', 'bool', 0, None, 'If the conveyor belt is curved, check this box to apply angular velocities. The rotation origin will be the rigid body origin.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:delta', 'float', 0, None, 'time since last step in seconds', {}, True, 0.0, False, ''),
        ('inputs:direction', 'float3', 0, None, 'relative direction to apply velocity', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0, 0.0]'}, True, [1.0, 0.0, 0.0], False, ''),
        ('inputs:enabled', 'bool', 0, None, 'node does not execute if disabled', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:onStep', 'execution', 0, None, 'step to animate textures', {}, True, None, False, ''),
        ('inputs:velocity', 'float', 0, None, 'the velocity of the conveyor unit', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.conveyorPrim = og.AttributeRole.TARGET
        role_data.inputs.onStep = og.AttributeRole.EXECUTION
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
        def animateDirection(self):
            data_view = og.AttributeValueHelper(self._attributes.animateDirection)
            return data_view.get()

        @animateDirection.setter
        def animateDirection(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.animateDirection)
            data_view = og.AttributeValueHelper(self._attributes.animateDirection)
            data_view.set(value)

        @property
        def animateScale(self):
            data_view = og.AttributeValueHelper(self._attributes.animateScale)
            return data_view.get()

        @animateScale.setter
        def animateScale(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.animateScale)
            data_view = og.AttributeValueHelper(self._attributes.animateScale)
            data_view.set(value)

        @property
        def animateTexture(self):
            data_view = og.AttributeValueHelper(self._attributes.animateTexture)
            return data_view.get()

        @animateTexture.setter
        def animateTexture(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.animateTexture)
            data_view = og.AttributeValueHelper(self._attributes.animateTexture)
            data_view.set(value)

        @property
        def conveyorPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.conveyorPrim)
            return data_view.get()

        @conveyorPrim.setter
        def conveyorPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.conveyorPrim)
            data_view = og.AttributeValueHelper(self._attributes.conveyorPrim)
            data_view.set(value)
            self.conveyorPrim_size = data_view.get_array_size()

        @property
        def curved(self):
            data_view = og.AttributeValueHelper(self._attributes.curved)
            return data_view.get()

        @curved.setter
        def curved(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.curved)
            data_view = og.AttributeValueHelper(self._attributes.curved)
            data_view.set(value)

        @property
        def delta(self):
            data_view = og.AttributeValueHelper(self._attributes.delta)
            return data_view.get()

        @delta.setter
        def delta(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.delta)
            data_view = og.AttributeValueHelper(self._attributes.delta)
            data_view.set(value)

        @property
        def direction(self):
            data_view = og.AttributeValueHelper(self._attributes.direction)
            return data_view.get()

        @direction.setter
        def direction(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.direction)
            data_view = og.AttributeValueHelper(self._attributes.direction)
            data_view.set(value)

        @property
        def enabled(self):
            data_view = og.AttributeValueHelper(self._attributes.enabled)
            return data_view.get()

        @enabled.setter
        def enabled(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.enabled)
            data_view = og.AttributeValueHelper(self._attributes.enabled)
            data_view.set(value)

        @property
        def onStep(self):
            data_view = og.AttributeValueHelper(self._attributes.onStep)
            return data_view.get()

        @onStep.setter
        def onStep(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.onStep)
            data_view = og.AttributeValueHelper(self._attributes.onStep)
            data_view.set(value)

        @property
        def velocity(self):
            data_view = og.AttributeValueHelper(self._attributes.velocity)
            return data_view.get()

        @velocity.setter
        def velocity(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.velocity)
            data_view = og.AttributeValueHelper(self._attributes.velocity)
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
        self.inputs = OgnIsaacConveyorDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacConveyorDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacConveyorDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
