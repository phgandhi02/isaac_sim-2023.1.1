"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishLaserScan

This node publishes LiDAR scans as a ROS2 LaserScan message
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2PublishLaserScanDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishLaserScan

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.azimuthRange
            inputs.context
            inputs.depthRange
            inputs.execIn
            inputs.frameId
            inputs.horizontalFov
            inputs.horizontalResolution
            inputs.intensitiesData
            inputs.linearDepthData
            inputs.nodeNamespace
            inputs.numCols
            inputs.numRows
            inputs.queueSize
            inputs.rotationRate
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
        ('inputs:azimuthRange', 'float2', 0, None, 'The azimuth range [min, max]', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0]'}, True, [0.0, 0.0], False, ''),
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:depthRange', 'float2', 0, None, 'The min and max range for sensor to detect a hit [min, max]', {ogn.MetadataKeys.DEFAULT: '[0, 0]'}, True, [0, 0], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message', {ogn.MetadataKeys.DEFAULT: '"sim_lidar"'}, True, "sim_lidar", False, ''),
        ('inputs:horizontalFov', 'float', 0, None, 'Horizontal Field of View in degrees', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:horizontalResolution', 'float', 0, None, 'Degrees in between rays for horizontal axis', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:intensitiesData', 'uchar[]', 0, None, 'Buffer array containing intensities data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:linearDepthData', 'float[]', 0, None, 'Buffer array containing linear depth data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:numCols', 'int', 0, None, 'Number of columns in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:numRows', 'int', 0, None, 'Number of rows in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:rotationRate', 'float', 0, None, 'Rotation rate of sensor in Hz', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS2 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"scan"'}, True, "scan", False, ''),
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
        def azimuthRange(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuthRange)
            return data_view.get()

        @azimuthRange.setter
        def azimuthRange(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.azimuthRange)
            data_view = og.AttributeValueHelper(self._attributes.azimuthRange)
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
        def depthRange(self):
            data_view = og.AttributeValueHelper(self._attributes.depthRange)
            return data_view.get()

        @depthRange.setter
        def depthRange(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.depthRange)
            data_view = og.AttributeValueHelper(self._attributes.depthRange)
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
        def horizontalFov(self):
            data_view = og.AttributeValueHelper(self._attributes.horizontalFov)
            return data_view.get()

        @horizontalFov.setter
        def horizontalFov(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.horizontalFov)
            data_view = og.AttributeValueHelper(self._attributes.horizontalFov)
            data_view.set(value)

        @property
        def horizontalResolution(self):
            data_view = og.AttributeValueHelper(self._attributes.horizontalResolution)
            return data_view.get()

        @horizontalResolution.setter
        def horizontalResolution(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.horizontalResolution)
            data_view = og.AttributeValueHelper(self._attributes.horizontalResolution)
            data_view.set(value)

        @property
        def intensitiesData(self):
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            return data_view.get()

        @intensitiesData.setter
        def intensitiesData(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.intensitiesData)
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            data_view.set(value)
            self.intensitiesData_size = data_view.get_array_size()

        @property
        def linearDepthData(self):
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            return data_view.get()

        @linearDepthData.setter
        def linearDepthData(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.linearDepthData)
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            data_view.set(value)
            self.linearDepthData_size = data_view.get_array_size()

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
        def numCols(self):
            data_view = og.AttributeValueHelper(self._attributes.numCols)
            return data_view.get()

        @numCols.setter
        def numCols(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.numCols)
            data_view = og.AttributeValueHelper(self._attributes.numCols)
            data_view.set(value)

        @property
        def numRows(self):
            data_view = og.AttributeValueHelper(self._attributes.numRows)
            return data_view.get()

        @numRows.setter
        def numRows(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.numRows)
            data_view = og.AttributeValueHelper(self._attributes.numRows)
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
        def rotationRate(self):
            data_view = og.AttributeValueHelper(self._attributes.rotationRate)
            return data_view.get()

        @rotationRate.setter
        def rotationRate(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.rotationRate)
            data_view = og.AttributeValueHelper(self._attributes.rotationRate)
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
        self.inputs = OgnROS2PublishLaserScanDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishLaserScanDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishLaserScanDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
