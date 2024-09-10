"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishOdometry

This node publishes odometry as a ROS2 Odometry message
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2PublishOdometryDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishOdometry

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.angularVelocity
            inputs.chassisFrameId
            inputs.context
            inputs.execIn
            inputs.linearVelocity
            inputs.nodeNamespace
            inputs.odomFrameId
            inputs.orientation
            inputs.position
            inputs.queueSize
            inputs.robotFront
            inputs.timeStamp
            inputs.topicName
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
        ('inputs:angularVelocity', 'vector3d', 0, None, 'Angular velocity vector in rad/s', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:chassisFrameId', 'string', 0, None, 'FrameId for robot chassis frame', {ogn.MetadataKeys.DEFAULT: '"base_link"'}, True, "base_link", False, ''),
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:linearVelocity', 'vector3d', 0, None, 'Linear velocity vector in m/s', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:odomFrameId', 'string', 0, None, 'FrameId for ROS2 odometry message', {ogn.MetadataKeys.DEFAULT: '"odom"'}, True, "odom", False, ''),
        ('inputs:orientation', 'quatd', 0, None, 'Orientation as a quaternion (IJKR)', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('inputs:position', 'vector3d', 0, None, 'Position vector in meters', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:robotFront', 'vector3d', 0, None, 'The front of the robot', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0, 0.0]'}, True, [1.0, 0.0, 0.0], False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS2 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"odom"'}, True, "odom", False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.angularVelocity = og.AttributeRole.VECTOR
        role_data.inputs.chassisFrameId = og.AttributeRole.TEXT
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.linearVelocity = og.AttributeRole.VECTOR
        role_data.inputs.nodeNamespace = og.AttributeRole.TEXT
        role_data.inputs.odomFrameId = og.AttributeRole.TEXT
        role_data.inputs.orientation = og.AttributeRole.QUATERNION
        role_data.inputs.position = og.AttributeRole.VECTOR
        role_data.inputs.robotFront = og.AttributeRole.VECTOR
        role_data.inputs.topicName = og.AttributeRole.TEXT
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
        def angularVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.angularVelocity)
            return data_view.get()

        @angularVelocity.setter
        def angularVelocity(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.angularVelocity)
            data_view = og.AttributeValueHelper(self._attributes.angularVelocity)
            data_view.set(value)

        @property
        def chassisFrameId(self):
            data_view = og.AttributeValueHelper(self._attributes.chassisFrameId)
            return data_view.get()

        @chassisFrameId.setter
        def chassisFrameId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.chassisFrameId)
            data_view = og.AttributeValueHelper(self._attributes.chassisFrameId)
            data_view.set(value)
            self.chassisFrameId_size = data_view.get_array_size()

        @property
        def context(self):
            data_view = og.AttributeValueHelper(self._attributes.context)
            return data_view.get()

        @context.setter
        def context(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.context)
            data_view = og.AttributeValueHelper(self._attributes.context)
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
        def linearVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.linearVelocity)
            return data_view.get()

        @linearVelocity.setter
        def linearVelocity(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.linearVelocity)
            data_view = og.AttributeValueHelper(self._attributes.linearVelocity)
            data_view.set(value)

        @property
        def nodeNamespace(self):
            data_view = og.AttributeValueHelper(self._attributes.nodeNamespace)
            return data_view.get()

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.nodeNamespace)
            data_view = og.AttributeValueHelper(self._attributes.nodeNamespace)
            data_view.set(value)
            self.nodeNamespace_size = data_view.get_array_size()

        @property
        def odomFrameId(self):
            data_view = og.AttributeValueHelper(self._attributes.odomFrameId)
            return data_view.get()

        @odomFrameId.setter
        def odomFrameId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.odomFrameId)
            data_view = og.AttributeValueHelper(self._attributes.odomFrameId)
            data_view.set(value)
            self.odomFrameId_size = data_view.get_array_size()

        @property
        def orientation(self):
            data_view = og.AttributeValueHelper(self._attributes.orientation)
            return data_view.get()

        @orientation.setter
        def orientation(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.orientation)
            data_view = og.AttributeValueHelper(self._attributes.orientation)
            data_view.set(value)

        @property
        def position(self):
            data_view = og.AttributeValueHelper(self._attributes.position)
            return data_view.get()

        @position.setter
        def position(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.position)
            data_view = og.AttributeValueHelper(self._attributes.position)
            data_view.set(value)

        @property
        def queueSize(self):
            data_view = og.AttributeValueHelper(self._attributes.queueSize)
            return data_view.get()

        @queueSize.setter
        def queueSize(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.queueSize)
            data_view = og.AttributeValueHelper(self._attributes.queueSize)
            data_view.set(value)

        @property
        def robotFront(self):
            data_view = og.AttributeValueHelper(self._attributes.robotFront)
            return data_view.get()

        @robotFront.setter
        def robotFront(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.robotFront)
            data_view = og.AttributeValueHelper(self._attributes.robotFront)
            data_view.set(value)

        @property
        def timeStamp(self):
            data_view = og.AttributeValueHelper(self._attributes.timeStamp)
            return data_view.get()

        @timeStamp.setter
        def timeStamp(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.timeStamp)
            data_view = og.AttributeValueHelper(self._attributes.timeStamp)
            data_view.set(value)

        @property
        def topicName(self):
            data_view = og.AttributeValueHelper(self._attributes.topicName)
            return data_view.get()

        @topicName.setter
        def topicName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.topicName)
            data_view = og.AttributeValueHelper(self._attributes.topicName)
            data_view.set(value)
            self.topicName_size = data_view.get_array_size()

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
        self.inputs = OgnROS2PublishOdometryDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishOdometryDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishOdometryDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
