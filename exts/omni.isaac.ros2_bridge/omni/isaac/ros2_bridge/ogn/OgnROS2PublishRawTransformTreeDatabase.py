"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishRawTransformTree

This node publishes a user-defined transformation between any two coordinate frames as a ROS2 Transform Tree
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2PublishRawTransformTreeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishRawTransformTree

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.childFrameId
            inputs.context
            inputs.execIn
            inputs.nodeNamespace
            inputs.parentFrameId
            inputs.queueSize
            inputs.rotation
            inputs.timeStamp
            inputs.topicName
            inputs.translation
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
        ('inputs:childFrameId', 'string', 0, None, 'Child frameId for ROS2 TF message', {ogn.MetadataKeys.DEFAULT: '"base_link"'}, True, "base_link", False, ''),
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:parentFrameId', 'string', 0, None, 'Parent frameId for ROS2 TF message', {ogn.MetadataKeys.DEFAULT: '"odom"'}, True, "odom", False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:rotation', 'quatd', 0, None, 'Rotation as a quaternion (IJKR)', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS2 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"tf"'}, True, "tf", False, ''),
        ('inputs:translation', 'vector3d', 0, None, 'Translation vector in meters', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.childFrameId = og.AttributeRole.TEXT
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.nodeNamespace = og.AttributeRole.TEXT
        role_data.inputs.parentFrameId = og.AttributeRole.TEXT
        role_data.inputs.rotation = og.AttributeRole.QUATERNION
        role_data.inputs.topicName = og.AttributeRole.TEXT
        role_data.inputs.translation = og.AttributeRole.VECTOR
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
        def childFrameId(self):
            data_view = og.AttributeValueHelper(self._attributes.childFrameId)
            return data_view.get()

        @childFrameId.setter
        def childFrameId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.childFrameId)
            data_view = og.AttributeValueHelper(self._attributes.childFrameId)
            data_view.set(value)
            self.childFrameId_size = data_view.get_array_size()

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
        def parentFrameId(self):
            data_view = og.AttributeValueHelper(self._attributes.parentFrameId)
            return data_view.get()

        @parentFrameId.setter
        def parentFrameId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.parentFrameId)
            data_view = og.AttributeValueHelper(self._attributes.parentFrameId)
            data_view.set(value)
            self.parentFrameId_size = data_view.get_array_size()

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
        def rotation(self):
            data_view = og.AttributeValueHelper(self._attributes.rotation)
            return data_view.get()

        @rotation.setter
        def rotation(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.rotation)
            data_view = og.AttributeValueHelper(self._attributes.rotation)
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

        @property
        def translation(self):
            data_view = og.AttributeValueHelper(self._attributes.translation)
            return data_view.get()

        @translation.setter
        def translation(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.translation)
            data_view = og.AttributeValueHelper(self._attributes.translation)
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
        self.inputs = OgnROS2PublishRawTransformTreeDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishRawTransformTreeDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishRawTransformTreeDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
