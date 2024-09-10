"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishImage

This node publishes ROS2 Image messages
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2PublishImageDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishImage

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.bufferSize
            inputs.context
            inputs.cudaDeviceIndex
            inputs.data
            inputs.dataPtr
            inputs.encoding
            inputs.execIn
            inputs.format
            inputs.frameId
            inputs.height
            inputs.nodeNamespace
            inputs.queueSize
            inputs.timeStamp
            inputs.topicName
            inputs.width

    Predefined Tokens:
        tokens.Type_RGB8
        tokens.Type_RGBA8
        tokens.Type_32FC1
        tokens.Type_32SC1
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
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('inputs:data', 'uchar[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'any', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:dataPtr', 'uint64', 0, None, 'Pointer to the raw rgba array data', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:encoding', 'token', 0, None, 'ROS encoding format for the input data, taken from the list of strings in include/sensor_msgs/image_encodings.h. Input data is expected to already be in this format, no conversions are performed', {ogn.MetadataKeys.ALLOWED_TOKENS: 'rgb8,rgba8,32FC1,32SC1', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"Type_RGB8": "rgb8", "Type_RGBA8": "rgba8", "Type_32FC1": "32FC1", "Type_32SC1": "32SC1"}', ogn.MetadataKeys.DEFAULT: '"rgb8"'}, True, "rgb8", False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port.', {}, True, None, False, ''),
        ('inputs:format', 'uint64', 0, None, 'Format', {}, True, 0, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, "sim_camera", False, ''),
        ('inputs:height', 'uint', 0, None, 'Buffer array height', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:timeStamp', 'double', 0, None, 'Time in seconds to use when publishing the message', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"rgb"'}, True, "rgb", False, ''),
        ('inputs:width', 'uint', 0, None, 'Buffer array width', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])

    class tokens:
        Type_RGB8 = "rgb8"
        Type_RGBA8 = "rgba8"
        Type_32FC1 = "32FC1"
        Type_32SC1 = "32SC1"

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.frameId = og.AttributeRole.TEXT
        role_data.inputs.nodeNamespace = og.AttributeRole.TEXT
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
        def cudaDeviceIndex(self):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            return data_view.get()

        @cudaDeviceIndex.setter
        def cudaDeviceIndex(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cudaDeviceIndex)
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            data_view.set(value)

        class __data:
            def __init__(self, parent):
                self._parent = parent

            @property
            def cpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                return data_view.get()

            @cpu.setter
            def cpu(self, value):
                if self._parent._setting_locked:
                    raise og.ReadOnlyError(self._parent._attributes.cpu)
                data_view = og.AttributeValueHelper(self._parent._attributes.cpu)
                data_view.set(value)
                self._parent.cpu_size = data_view.get_array_size()

            @property
            def gpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                return data_view.get(on_gpu=True)

            @gpu.setter
            def gpu(self, value):
                if self._parent._setting_locked:
                    raise og.ReadOnlyError(self._parent._attributes.gpu)
                data_view = og.AttributeValueHelper(self._parent._attributes.gpu)
                data_view.set(value)
                self._parent.gpu_size = data_view.get_array_size()

        @property
        def data(self):
            return self.__class__.__data(self)

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
        def encoding(self):
            data_view = og.AttributeValueHelper(self._attributes.encoding)
            return data_view.get()

        @encoding.setter
        def encoding(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.encoding)
            data_view = og.AttributeValueHelper(self._attributes.encoding)
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
        def frameId(self):
            data_view = og.AttributeValueHelper(self._attributes.frameId)
            return data_view.get()

        @frameId.setter
        def frameId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.frameId)
            data_view = og.AttributeValueHelper(self._attributes.frameId)
            data_view.set(value)
            self.frameId_size = data_view.get_array_size()

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
        self.inputs = OgnROS2PublishImageDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishImageDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishImageDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
