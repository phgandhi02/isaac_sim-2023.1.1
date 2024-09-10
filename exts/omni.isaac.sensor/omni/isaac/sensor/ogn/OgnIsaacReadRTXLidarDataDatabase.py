"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacReadRTXLidarData

This node reads the data straight from the an RTX Lidar sensor.
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacReadRTXLidarDataDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacReadRTXLidarData

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.dataPtr
            inputs.exec
            inputs.keepOnlyPositiveDistance
            inputs.renderProductPath
        Outputs:
            outputs.azimuths
            outputs.beamIds
            outputs.channels
            outputs.deltaTimes
            outputs.depthRange
            outputs.distances
            outputs.echos
            outputs.elevations
            outputs.emitterIds
            outputs.exec
            outputs.hitPointNormals
            outputs.intensities
            outputs.materialIds
            outputs.numBeams
            outputs.numChannels
            outputs.numEchos
            outputs.numTicks
            outputs.objectIds
            outputs.tickAzimuths
            outputs.tickStates
            outputs.tickTimestamps
            outputs.ticks
            outputs.velocities
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
        ('inputs:dataPtr', 'uint64', 0, 'Data Pointer', 'Pointer to LiDAR render result.', {}, True, 0, False, ''),
        ('inputs:exec', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:keepOnlyPositiveDistance', 'bool', 0, 'Keep Only Positive Distance', 'Keep points only if the return distance is > 0', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Config is gotten from this', {}, True, "", False, ''),
        ('outputs:azimuths', 'float[]', 0, None, 'azimuth in deg [0, 360)', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:beamIds', 'uint[]', 0, None, 'beam/laser detector id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:channels', 'uint[]', 0, None, 'channel of point', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:deltaTimes', 'uint[]', 0, None, 'delta time in ns from the head (relative to tick timestamp)', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:depthRange', 'float2', 0, None, 'The min and max range for sensor to detect a hit [min, max]', {ogn.MetadataKeys.DEFAULT: '[0, 0]'}, True, [0, 0], False, ''),
        ('outputs:distances', 'float[]', 0, None, 'distance in m', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:echos', 'uint[]', 0, None, 'echo id in ascending order', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:elevations', 'float[]', 0, None, 'elevation in deg [-90, 90]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:emitterIds', 'uint[]', 0, None, 'beam/laser detector id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:exec', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:hitPointNormals', 'point3f[]', 0, None, 'hit point Normal', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:intensities', 'float[]', 0, None, 'intensity [0,1]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:materialIds', 'uint[]', 0, None, 'hit point material id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:numBeams', 'uint64', 0, None, 'The number of lidar beams being output', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:numChannels', 'uint', 0, None, 'The number of emitter firings per tick', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:numEchos', 'uint', 0, None, 'The number of returns per single channel firing', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:numTicks', 'uint', 0, None, 'The number of ticks in the output', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:objectIds', 'uint[]', 0, None, 'hit point object id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:tickAzimuths', 'float[]', 0, None, 'azimuth origin of a tick in deg [0, 360)', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:tickStates', 'uint[]', 0, None, 'emitter state the tick belongs to', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:tickTimestamps', 'uint64[]', 0, None, 'timestamp of the tick in Ns', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:ticks', 'uint[]', 0, None, 'tick of point', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:velocities', 'point3f[]', 0, None, 'velocity at hit point in sensor coordinates [m/s]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.hitPointNormals = og.AttributeRole.POSITION
        role_data.outputs.velocities = og.AttributeRole.POSITION
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
        def exec(self):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            return data_view.get()

        @exec.setter
        def exec(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.exec)
            data_view = og.AttributeValueHelper(self._attributes.exec)
            data_view.set(value)

        @property
        def keepOnlyPositiveDistance(self):
            data_view = og.AttributeValueHelper(self._attributes.keepOnlyPositiveDistance)
            return data_view.get()

        @keepOnlyPositiveDistance.setter
        def keepOnlyPositiveDistance(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.keepOnlyPositiveDistance)
            data_view = og.AttributeValueHelper(self._attributes.keepOnlyPositiveDistance)
            data_view.set(value)

        @property
        def renderProductPath(self):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
            return data_view.get()

        @renderProductPath.setter
        def renderProductPath(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.renderProductPath)
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
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
            self.azimuths_size = 0
            self.beamIds_size = 0
            self.channels_size = 0
            self.deltaTimes_size = 0
            self.distances_size = 0
            self.echos_size = 0
            self.elevations_size = 0
            self.emitterIds_size = 0
            self.hitPointNormals_size = 0
            self.intensities_size = 0
            self.materialIds_size = 0
            self.objectIds_size = 0
            self.tickAzimuths_size = 0
            self.tickStates_size = 0
            self.tickTimestamps_size = 0
            self.ticks_size = 0
            self.velocities_size = 0
            self._batchedWriteValues = { }

        @property
        def azimuths(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuths)
            return data_view.get(reserved_element_count=self.azimuths_size)

        @azimuths.setter
        def azimuths(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuths)
            data_view.set(value)
            self.azimuths_size = data_view.get_array_size()

        @property
        def beamIds(self):
            data_view = og.AttributeValueHelper(self._attributes.beamIds)
            return data_view.get(reserved_element_count=self.beamIds_size)

        @beamIds.setter
        def beamIds(self, value):
            data_view = og.AttributeValueHelper(self._attributes.beamIds)
            data_view.set(value)
            self.beamIds_size = data_view.get_array_size()

        @property
        def channels(self):
            data_view = og.AttributeValueHelper(self._attributes.channels)
            return data_view.get(reserved_element_count=self.channels_size)

        @channels.setter
        def channels(self, value):
            data_view = og.AttributeValueHelper(self._attributes.channels)
            data_view.set(value)
            self.channels_size = data_view.get_array_size()

        @property
        def deltaTimes(self):
            data_view = og.AttributeValueHelper(self._attributes.deltaTimes)
            return data_view.get(reserved_element_count=self.deltaTimes_size)

        @deltaTimes.setter
        def deltaTimes(self, value):
            data_view = og.AttributeValueHelper(self._attributes.deltaTimes)
            data_view.set(value)
            self.deltaTimes_size = data_view.get_array_size()

        @property
        def depthRange(self):
            data_view = og.AttributeValueHelper(self._attributes.depthRange)
            return data_view.get()

        @depthRange.setter
        def depthRange(self, value):
            data_view = og.AttributeValueHelper(self._attributes.depthRange)
            data_view.set(value)

        @property
        def distances(self):
            data_view = og.AttributeValueHelper(self._attributes.distances)
            return data_view.get(reserved_element_count=self.distances_size)

        @distances.setter
        def distances(self, value):
            data_view = og.AttributeValueHelper(self._attributes.distances)
            data_view.set(value)
            self.distances_size = data_view.get_array_size()

        @property
        def echos(self):
            data_view = og.AttributeValueHelper(self._attributes.echos)
            return data_view.get(reserved_element_count=self.echos_size)

        @echos.setter
        def echos(self, value):
            data_view = og.AttributeValueHelper(self._attributes.echos)
            data_view.set(value)
            self.echos_size = data_view.get_array_size()

        @property
        def elevations(self):
            data_view = og.AttributeValueHelper(self._attributes.elevations)
            return data_view.get(reserved_element_count=self.elevations_size)

        @elevations.setter
        def elevations(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevations)
            data_view.set(value)
            self.elevations_size = data_view.get_array_size()

        @property
        def emitterIds(self):
            data_view = og.AttributeValueHelper(self._attributes.emitterIds)
            return data_view.get(reserved_element_count=self.emitterIds_size)

        @emitterIds.setter
        def emitterIds(self, value):
            data_view = og.AttributeValueHelper(self._attributes.emitterIds)
            data_view.set(value)
            self.emitterIds_size = data_view.get_array_size()

        @property
        def exec(self):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            return data_view.get()

        @exec.setter
        def exec(self, value):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            data_view.set(value)

        @property
        def hitPointNormals(self):
            data_view = og.AttributeValueHelper(self._attributes.hitPointNormals)
            return data_view.get(reserved_element_count=self.hitPointNormals_size)

        @hitPointNormals.setter
        def hitPointNormals(self, value):
            data_view = og.AttributeValueHelper(self._attributes.hitPointNormals)
            data_view.set(value)
            self.hitPointNormals_size = data_view.get_array_size()

        @property
        def intensities(self):
            data_view = og.AttributeValueHelper(self._attributes.intensities)
            return data_view.get(reserved_element_count=self.intensities_size)

        @intensities.setter
        def intensities(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensities)
            data_view.set(value)
            self.intensities_size = data_view.get_array_size()

        @property
        def materialIds(self):
            data_view = og.AttributeValueHelper(self._attributes.materialIds)
            return data_view.get(reserved_element_count=self.materialIds_size)

        @materialIds.setter
        def materialIds(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialIds)
            data_view.set(value)
            self.materialIds_size = data_view.get_array_size()

        @property
        def numBeams(self):
            data_view = og.AttributeValueHelper(self._attributes.numBeams)
            return data_view.get()

        @numBeams.setter
        def numBeams(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numBeams)
            data_view.set(value)

        @property
        def numChannels(self):
            data_view = og.AttributeValueHelper(self._attributes.numChannels)
            return data_view.get()

        @numChannels.setter
        def numChannels(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numChannels)
            data_view.set(value)

        @property
        def numEchos(self):
            data_view = og.AttributeValueHelper(self._attributes.numEchos)
            return data_view.get()

        @numEchos.setter
        def numEchos(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numEchos)
            data_view.set(value)

        @property
        def numTicks(self):
            data_view = og.AttributeValueHelper(self._attributes.numTicks)
            return data_view.get()

        @numTicks.setter
        def numTicks(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numTicks)
            data_view.set(value)

        @property
        def objectIds(self):
            data_view = og.AttributeValueHelper(self._attributes.objectIds)
            return data_view.get(reserved_element_count=self.objectIds_size)

        @objectIds.setter
        def objectIds(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectIds)
            data_view.set(value)
            self.objectIds_size = data_view.get_array_size()

        @property
        def tickAzimuths(self):
            data_view = og.AttributeValueHelper(self._attributes.tickAzimuths)
            return data_view.get(reserved_element_count=self.tickAzimuths_size)

        @tickAzimuths.setter
        def tickAzimuths(self, value):
            data_view = og.AttributeValueHelper(self._attributes.tickAzimuths)
            data_view.set(value)
            self.tickAzimuths_size = data_view.get_array_size()

        @property
        def tickStates(self):
            data_view = og.AttributeValueHelper(self._attributes.tickStates)
            return data_view.get(reserved_element_count=self.tickStates_size)

        @tickStates.setter
        def tickStates(self, value):
            data_view = og.AttributeValueHelper(self._attributes.tickStates)
            data_view.set(value)
            self.tickStates_size = data_view.get_array_size()

        @property
        def tickTimestamps(self):
            data_view = og.AttributeValueHelper(self._attributes.tickTimestamps)
            return data_view.get(reserved_element_count=self.tickTimestamps_size)

        @tickTimestamps.setter
        def tickTimestamps(self, value):
            data_view = og.AttributeValueHelper(self._attributes.tickTimestamps)
            data_view.set(value)
            self.tickTimestamps_size = data_view.get_array_size()

        @property
        def ticks(self):
            data_view = og.AttributeValueHelper(self._attributes.ticks)
            return data_view.get(reserved_element_count=self.ticks_size)

        @ticks.setter
        def ticks(self, value):
            data_view = og.AttributeValueHelper(self._attributes.ticks)
            data_view.set(value)
            self.ticks_size = data_view.get_array_size()

        @property
        def velocities(self):
            data_view = og.AttributeValueHelper(self._attributes.velocities)
            return data_view.get(reserved_element_count=self.velocities_size)

        @velocities.setter
        def velocities(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocities)
            data_view.set(value)
            self.velocities_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacReadRTXLidarDataDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadRTXLidarDataDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadRTXLidarDataDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
