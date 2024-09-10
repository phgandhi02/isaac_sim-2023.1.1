"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacComputeOdometry

Holds values related to odometry, this node is not a replcement for the IMU sensor and the associated Read IMU node
"""

import carb
import numpy
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacComputeOdometryDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacComputeOdometry

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.chassisPrim
            inputs.execIn
        Outputs:
            outputs.angularAcceleration
            outputs.angularVelocity
            outputs.execOut
            outputs.linearAcceleration
            outputs.linearVelocity
            outputs.orientation
            outputs.position
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
        ('inputs:chassisPrim', 'target', 0, None, 'Usd prim reference to the articulation root or rigid body prim', {}, True, [], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('outputs:angularAcceleration', 'vector3d', 0, None, 'Angular acceleration vector in rad/s^2', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:angularVelocity', 'vector3d', 0, None, 'Angular velocity vector in rad/s', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'The output execution port', {}, True, None, False, ''),
        ('outputs:linearAcceleration', 'vector3d', 0, None, 'Linear acceleration vector in m/s^2', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:linearVelocity', 'vector3d', 0, None, 'Linear velocity vector in m/s', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:orientation', 'quatd', 0, None, 'Rotation as a quaternion (IJKR)', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('outputs:position', 'vector3d', 0, None, 'Position vector in meters', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.chassisPrim = og.AttributeRole.TARGET
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.angularAcceleration = og.AttributeRole.VECTOR
        role_data.outputs.angularVelocity = og.AttributeRole.VECTOR
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        role_data.outputs.linearAcceleration = og.AttributeRole.VECTOR
        role_data.outputs.linearVelocity = og.AttributeRole.VECTOR
        role_data.outputs.orientation = og.AttributeRole.QUATERNION
        role_data.outputs.position = og.AttributeRole.VECTOR
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
        def chassisPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.chassisPrim)
            return data_view.get()

        @chassisPrim.setter
        def chassisPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.chassisPrim)
            data_view = og.AttributeValueHelper(self._attributes.chassisPrim)
            data_view.set(value)
            self.chassisPrim_size = data_view.get_array_size()

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
        def angularAcceleration(self):
            data_view = og.AttributeValueHelper(self._attributes.angularAcceleration)
            return data_view.get()

        @angularAcceleration.setter
        def angularAcceleration(self, value):
            data_view = og.AttributeValueHelper(self._attributes.angularAcceleration)
            data_view.set(value)

        @property
        def angularVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.angularVelocity)
            return data_view.get()

        @angularVelocity.setter
        def angularVelocity(self, value):
            data_view = og.AttributeValueHelper(self._attributes.angularVelocity)
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
        def linearAcceleration(self):
            data_view = og.AttributeValueHelper(self._attributes.linearAcceleration)
            return data_view.get()

        @linearAcceleration.setter
        def linearAcceleration(self, value):
            data_view = og.AttributeValueHelper(self._attributes.linearAcceleration)
            data_view.set(value)

        @property
        def linearVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.linearVelocity)
            return data_view.get()

        @linearVelocity.setter
        def linearVelocity(self, value):
            data_view = og.AttributeValueHelper(self._attributes.linearVelocity)
            data_view.set(value)

        @property
        def orientation(self):
            data_view = og.AttributeValueHelper(self._attributes.orientation)
            return data_view.get()

        @orientation.setter
        def orientation(self, value):
            data_view = og.AttributeValueHelper(self._attributes.orientation)
            data_view.set(value)

        @property
        def position(self):
            data_view = og.AttributeValueHelper(self._attributes.position)
            return data_view.get()

        @position.setter
        def position(self, value):
            data_view = og.AttributeValueHelper(self._attributes.position)
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
        self.inputs = OgnIsaacComputeOdometryDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacComputeOdometryDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacComputeOdometryDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
