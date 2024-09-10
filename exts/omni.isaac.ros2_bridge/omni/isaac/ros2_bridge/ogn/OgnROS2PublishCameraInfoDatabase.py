"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishCameraInfo

This node publishes camera info as a ROS2 CameraInfo message
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2PublishCameraInfoDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishCameraInfo

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.context
            inputs.execIn
            inputs.focalLength
            inputs.frameId
            inputs.height
            inputs.horizontalAperture
            inputs.horizontalOffset
            inputs.nodeNamespace
            inputs.physicalDistortionCoefficients
            inputs.physicalDistortionModel
            inputs.projectionType
            inputs.queueSize
            inputs.stereoOffset
            inputs.timeStamp
            inputs.topicName
            inputs.verticalAperture
            inputs.verticalOffset
            inputs.width
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
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:focalLength', 'float', 0, None, 'focal length', {}, True, 0.0, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, "sim_camera", False, ''),
        ('inputs:height', 'uint', 0, None, 'Height for output image', {}, True, 0, False, ''),
        ('inputs:horizontalAperture', 'float', 0, None, 'horizontal aperture', {}, True, 0.0, False, ''),
        ('inputs:horizontalOffset', 'float', 0, None, 'horizontal offset', {}, True, 0.0, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:physicalDistortionCoefficients', 'float[]', 0, None, 'physical distortion model used for approximation, physicalDistortionModel must be set to use these coefficients', {}, True, [], False, ''),
        ('inputs:physicalDistortionModel', 'token', 0, None, 'physical distortion model used for approximation, if blank projectionType is used', {}, True, "", False, ''),
        ('inputs:projectionType', 'token', 0, None, 'projection type', {}, True, "", False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:stereoOffset', 'float2', 0, None, 'Stereo offset (Tx, Ty) used when publishing the camera info topic', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0]'}, True, [0.0, 0.0], False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS2 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"camera_info"'}, True, "camera_info", False, ''),
        ('inputs:verticalAperture', 'float', 0, None, 'vertical aperture', {}, True, 0.0, False, ''),
        ('inputs:verticalOffset', 'float', 0, None, 'vertical offset', {}, True, 0.0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width for output image', {}, True, 0, False, ''),
    ])

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
        def focalLength(self):
            data_view = og.AttributeValueHelper(self._attributes.focalLength)
            return data_view.get()

        @focalLength.setter
        def focalLength(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.focalLength)
            data_view = og.AttributeValueHelper(self._attributes.focalLength)
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
        def horizontalAperture(self):
            data_view = og.AttributeValueHelper(self._attributes.horizontalAperture)
            return data_view.get()

        @horizontalAperture.setter
        def horizontalAperture(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.horizontalAperture)
            data_view = og.AttributeValueHelper(self._attributes.horizontalAperture)
            data_view.set(value)

        @property
        def horizontalOffset(self):
            data_view = og.AttributeValueHelper(self._attributes.horizontalOffset)
            return data_view.get()

        @horizontalOffset.setter
        def horizontalOffset(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.horizontalOffset)
            data_view = og.AttributeValueHelper(self._attributes.horizontalOffset)
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
        def physicalDistortionCoefficients(self):
            data_view = og.AttributeValueHelper(self._attributes.physicalDistortionCoefficients)
            return data_view.get()

        @physicalDistortionCoefficients.setter
        def physicalDistortionCoefficients(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.physicalDistortionCoefficients)
            data_view = og.AttributeValueHelper(self._attributes.physicalDistortionCoefficients)
            data_view.set(value)
            self.physicalDistortionCoefficients_size = data_view.get_array_size()

        @property
        def physicalDistortionModel(self):
            data_view = og.AttributeValueHelper(self._attributes.physicalDistortionModel)
            return data_view.get()

        @physicalDistortionModel.setter
        def physicalDistortionModel(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.physicalDistortionModel)
            data_view = og.AttributeValueHelper(self._attributes.physicalDistortionModel)
            data_view.set(value)

        @property
        def projectionType(self):
            data_view = og.AttributeValueHelper(self._attributes.projectionType)
            return data_view.get()

        @projectionType.setter
        def projectionType(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.projectionType)
            data_view = og.AttributeValueHelper(self._attributes.projectionType)
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
        def stereoOffset(self):
            data_view = og.AttributeValueHelper(self._attributes.stereoOffset)
            return data_view.get()

        @stereoOffset.setter
        def stereoOffset(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.stereoOffset)
            data_view = og.AttributeValueHelper(self._attributes.stereoOffset)
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
        def verticalAperture(self):
            data_view = og.AttributeValueHelper(self._attributes.verticalAperture)
            return data_view.get()

        @verticalAperture.setter
        def verticalAperture(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.verticalAperture)
            data_view = og.AttributeValueHelper(self._attributes.verticalAperture)
            data_view.set(value)

        @property
        def verticalOffset(self):
            data_view = og.AttributeValueHelper(self._attributes.verticalOffset)
            return data_view.get()

        @verticalOffset.setter
        def verticalOffset(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.verticalOffset)
            data_view = og.AttributeValueHelper(self._attributes.verticalOffset)
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
        self.inputs = OgnROS2PublishCameraInfoDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishCameraInfoDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishCameraInfoDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
