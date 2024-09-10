"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacCreateRTXLidarScanBuffer

This node creates a full scan buffer for RTX Lidar sensor.
"""

import carb
import numpy

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacCreateRTXLidarScanBufferDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacCreateRTXLidarScanBuffer

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.accuracyErrorAzimuthDeg
            inputs.accuracyErrorElevationDeg
            inputs.accuracyErrorPosition
            inputs.cudaDeviceIndex
            inputs.dataPtr
            inputs.exec
            inputs.keepOnlyPositiveDistance
            inputs.outputAzimuth
            inputs.outputBeamId
            inputs.outputDistance
            inputs.outputElevation
            inputs.outputEmitterId
            inputs.outputIntensity
            inputs.outputMaterialId
            inputs.outputNormal
            inputs.outputObjectId
            inputs.outputTimestamp
            inputs.outputVelocity
            inputs.renderProductPath
            inputs.transformPoints
        Outputs:
            outputs.azimuthBufferSize
            outputs.azimuthDataType
            outputs.azimuthPtr
            outputs.beamIdBufferSize
            outputs.beamIdDataType
            outputs.beamIdPtr
            outputs.bufferSize
            outputs.cudaDeviceIndex
            outputs.dataPtr
            outputs.distanceBufferSize
            outputs.distanceDataType
            outputs.distancePtr
            outputs.elevationBufferSize
            outputs.elevationDataType
            outputs.elevationPtr
            outputs.emitterIdBufferSize
            outputs.emitterIdDataType
            outputs.emitterIdPtr
            outputs.exec
            outputs.height
            outputs.indexBufferSize
            outputs.indexDataType
            outputs.indexPtr
            outputs.intensityBufferSize
            outputs.intensityDataType
            outputs.intensityPtr
            outputs.materialIdBufferSize
            outputs.materialIdDataType
            outputs.materialIdPtr
            outputs.normalBufferSize
            outputs.normalDataType
            outputs.normalPtr
            outputs.numChannels
            outputs.numEchos
            outputs.numReturnsPerScan
            outputs.objectIdBufferSize
            outputs.objectIdDataType
            outputs.objectIdPtr
            outputs.renderProductPath
            outputs.ticksPerScan
            outputs.timestampBufferSize
            outputs.timestampDataType
            outputs.timestampPtr
            outputs.transform
            outputs.velocityBufferSize
            outputs.velocityDataType
            outputs.velocityPtr
            outputs.width
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
        ('inputs:accuracyErrorAzimuthDeg', 'float', 0, 'Error Azimuth', 'Accuracy error of azimuth in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorElevationDeg', 'float', 0, 'Error Elevation', 'Accuracy error of elevation in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorPosition', 'float3', 0, 'Error Position', 'Position offset applied to all points equally', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('inputs:dataPtr', 'uint64', 0, 'Data Pointer', 'Pointer to LiDAR render result.', {}, True, 0, False, ''),
        ('inputs:exec', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:keepOnlyPositiveDistance', 'bool', 0, 'Keep Only Positive Distance', 'Keep points only if the return distance is > 0', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:outputAzimuth', 'bool', 0, 'Output The Azimuth', 'Create an output array for the Azimuth.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputBeamId', 'bool', 0, 'Output The BeamId', 'Create an output array for the BeamId.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputDistance', 'bool', 0, 'Output The Distance', 'Create an output array for the Distance.', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:outputElevation', 'bool', 0, 'Output The Elevation', 'Create an output array for the Elevation.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputEmitterId', 'bool', 0, 'Output The EmitterId', 'Create an output array for the EmitterId.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputIntensity', 'bool', 0, 'Output The Intensity', 'Create an output array for the Intensity.', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:outputMaterialId', 'bool', 0, 'Output The MaterialId', 'Create an output array for the MaterialId.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputNormal', 'bool', 0, 'Output The Normals', 'Create an output array for the Normals.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputObjectId', 'bool', 0, 'Output The ObjectId', 'Create an output array for the ObjectId.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputTimestamp', 'bool', 0, 'Output The Timestamp', 'Create an output array for the Timestamp.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:outputVelocity', 'bool', 0, 'Output The Velocity', 'Create an output array for the Velocity.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Config is gotten from this', {}, True, "", False, ''),
        ('inputs:transformPoints', 'bool', 0, 'Output in World Coordinates', 'Transform point cloud to world coordinates', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('outputs:azimuthBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:azimuthDataType', 'float', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:azimuthPtr', 'uint64', 0, 'azimuth', 'azimuth in rad [-pi,pi]', {}, True, None, False, ''),
        ('outputs:beamIdBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:beamIdDataType', 'uint', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:beamIdPtr', 'uint64', 0, 'beamId', 'beamId', {}, True, None, False, ''),
        ('outputs:bufferSize', 'uint64', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:cudaDeviceIndex', 'int', 0, None, 'Index of the device where the data lives (-1 for host data)', {ogn.MetadataKeys.DEFAULT: '-1'}, True, -1, False, ''),
        ('outputs:dataPtr', 'uint64', 0, 'Point Cloud Data', 'Pointer to LiDAR render result.', {}, True, None, False, ''),
        ('outputs:distanceBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:distanceDataType', 'float', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:distancePtr', 'uint64', 0, 'distance', 'range in m', {}, True, None, False, ''),
        ('outputs:elevationBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:elevationDataType', 'float', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:elevationPtr', 'uint64', 0, 'elevation', 'elevation in rad [-pi/2, pi/2]', {}, True, None, False, ''),
        ('outputs:emitterIdBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:emitterIdDataType', 'uint', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:emitterIdPtr', 'uint64', 0, 'emitterId', 'emitterId', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height of point cloud buffer, will always return 1', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('outputs:indexBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:indexDataType', 'uint', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:indexPtr', 'uint64', 0, 'index', 'Index into the full array if keepOnlyPositiveDistance ((startTick+tick)*numChannels*numEchos + channel*numEchos + echo)', {}, True, None, False, ''),
        ('outputs:intensityBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:intensityDataType', 'float', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:intensityPtr', 'uint64', 0, 'intensity', 'intensity [0,1]', {}, True, None, False, ''),
        ('outputs:materialIdBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:materialIdDataType', 'uint', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:materialIdPtr', 'uint64', 0, 'materialId', 'materialId at hit location', {}, True, None, False, ''),
        ('outputs:normalBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:normalDataType', 'float3', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '[4, 0, 0]'}, True, [4, 0, 0], False, ''),
        ('outputs:normalPtr', 'uint64', 0, 'normal', 'Normal at the hit location', {}, True, None, False, ''),
        ('outputs:numChannels', 'uint', 0, None, 'Number of channels of the lidar', {}, True, None, False, ''),
        ('outputs:numEchos', 'uint', 0, None, 'Number of echos of the lidar', {}, True, None, False, ''),
        ('outputs:numReturnsPerScan', 'uint', 0, None, 'Number of returns in the full scan', {}, True, None, False, ''),
        ('outputs:objectIdBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:objectIdDataType', 'uint', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '4'}, True, 4, False, ''),
        ('outputs:objectIdPtr', 'uint64', 0, 'objectId', 'ObjectId for getting usd prim information', {}, True, None, False, ''),
        ('outputs:renderProductPath', 'token', 0, None, 'Config is gotten from this', {}, True, None, False, ''),
        ('outputs:ticksPerScan', 'uint', 0, None, 'Number of ticks in a full scan', {}, True, None, False, ''),
        ('outputs:timestampBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:timestampDataType', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '8'}, True, 8, False, ''),
        ('outputs:timestampPtr', 'uint64', 0, 'timestamp', 'timestamp in ns', {}, True, None, False, ''),
        ('outputs:transform', 'matrix4d', 0, None, 'The transform matrix from lidar to world coordinates', {}, True, None, False, ''),
        ('outputs:velocityBufferSize', 'uint64', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true'}, True, None, False, ''),
        ('outputs:velocityDataType', 'float3', 0, None, '', {ogn.MetadataKeys.HIDDEN: 'true', ogn.MetadataKeys.DEFAULT: '[4, 0, 0]'}, True, [4, 0, 0], False, ''),
        ('outputs:velocityPtr', 'uint64', 0, 'velocity', 'elevation in rad [-pi/2, pi/2]', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, '3 x Width or number of points in point cloud buffer', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.exec = og.AttributeRole.EXECUTION
        role_data.outputs.transform = og.AttributeRole.MATRIX
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
        def accuracyErrorAzimuthDeg(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorAzimuthDeg)
            return data_view.get()

        @accuracyErrorAzimuthDeg.setter
        def accuracyErrorAzimuthDeg(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorAzimuthDeg)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorAzimuthDeg)
            data_view.set(value)

        @property
        def accuracyErrorElevationDeg(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorElevationDeg)
            return data_view.get()

        @accuracyErrorElevationDeg.setter
        def accuracyErrorElevationDeg(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorElevationDeg)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorElevationDeg)
            data_view.set(value)

        @property
        def accuracyErrorPosition(self):
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorPosition)
            return data_view.get()

        @accuracyErrorPosition.setter
        def accuracyErrorPosition(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.accuracyErrorPosition)
            data_view = og.AttributeValueHelper(self._attributes.accuracyErrorPosition)
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
        def outputAzimuth(self):
            data_view = og.AttributeValueHelper(self._attributes.outputAzimuth)
            return data_view.get()

        @outputAzimuth.setter
        def outputAzimuth(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputAzimuth)
            data_view = og.AttributeValueHelper(self._attributes.outputAzimuth)
            data_view.set(value)

        @property
        def outputBeamId(self):
            data_view = og.AttributeValueHelper(self._attributes.outputBeamId)
            return data_view.get()

        @outputBeamId.setter
        def outputBeamId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputBeamId)
            data_view = og.AttributeValueHelper(self._attributes.outputBeamId)
            data_view.set(value)

        @property
        def outputDistance(self):
            data_view = og.AttributeValueHelper(self._attributes.outputDistance)
            return data_view.get()

        @outputDistance.setter
        def outputDistance(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputDistance)
            data_view = og.AttributeValueHelper(self._attributes.outputDistance)
            data_view.set(value)

        @property
        def outputElevation(self):
            data_view = og.AttributeValueHelper(self._attributes.outputElevation)
            return data_view.get()

        @outputElevation.setter
        def outputElevation(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputElevation)
            data_view = og.AttributeValueHelper(self._attributes.outputElevation)
            data_view.set(value)

        @property
        def outputEmitterId(self):
            data_view = og.AttributeValueHelper(self._attributes.outputEmitterId)
            return data_view.get()

        @outputEmitterId.setter
        def outputEmitterId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputEmitterId)
            data_view = og.AttributeValueHelper(self._attributes.outputEmitterId)
            data_view.set(value)

        @property
        def outputIntensity(self):
            data_view = og.AttributeValueHelper(self._attributes.outputIntensity)
            return data_view.get()

        @outputIntensity.setter
        def outputIntensity(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputIntensity)
            data_view = og.AttributeValueHelper(self._attributes.outputIntensity)
            data_view.set(value)

        @property
        def outputMaterialId(self):
            data_view = og.AttributeValueHelper(self._attributes.outputMaterialId)
            return data_view.get()

        @outputMaterialId.setter
        def outputMaterialId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputMaterialId)
            data_view = og.AttributeValueHelper(self._attributes.outputMaterialId)
            data_view.set(value)

        @property
        def outputNormal(self):
            data_view = og.AttributeValueHelper(self._attributes.outputNormal)
            return data_view.get()

        @outputNormal.setter
        def outputNormal(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputNormal)
            data_view = og.AttributeValueHelper(self._attributes.outputNormal)
            data_view.set(value)

        @property
        def outputObjectId(self):
            data_view = og.AttributeValueHelper(self._attributes.outputObjectId)
            return data_view.get()

        @outputObjectId.setter
        def outputObjectId(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputObjectId)
            data_view = og.AttributeValueHelper(self._attributes.outputObjectId)
            data_view.set(value)

        @property
        def outputTimestamp(self):
            data_view = og.AttributeValueHelper(self._attributes.outputTimestamp)
            return data_view.get()

        @outputTimestamp.setter
        def outputTimestamp(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputTimestamp)
            data_view = og.AttributeValueHelper(self._attributes.outputTimestamp)
            data_view.set(value)

        @property
        def outputVelocity(self):
            data_view = og.AttributeValueHelper(self._attributes.outputVelocity)
            return data_view.get()

        @outputVelocity.setter
        def outputVelocity(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.outputVelocity)
            data_view = og.AttributeValueHelper(self._attributes.outputVelocity)
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

        @property
        def transformPoints(self):
            data_view = og.AttributeValueHelper(self._attributes.transformPoints)
            return data_view.get()

        @transformPoints.setter
        def transformPoints(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.transformPoints)
            data_view = og.AttributeValueHelper(self._attributes.transformPoints)
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
        def azimuthBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuthBufferSize)
            return data_view.get()

        @azimuthBufferSize.setter
        def azimuthBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuthBufferSize)
            data_view.set(value)

        @property
        def azimuthDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuthDataType)
            return data_view.get()

        @azimuthDataType.setter
        def azimuthDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuthDataType)
            data_view.set(value)

        @property
        def azimuthPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuthPtr)
            return data_view.get()

        @azimuthPtr.setter
        def azimuthPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuthPtr)
            data_view.set(value)

        @property
        def beamIdBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.beamIdBufferSize)
            return data_view.get()

        @beamIdBufferSize.setter
        def beamIdBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.beamIdBufferSize)
            data_view.set(value)

        @property
        def beamIdDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.beamIdDataType)
            return data_view.get()

        @beamIdDataType.setter
        def beamIdDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.beamIdDataType)
            data_view.set(value)

        @property
        def beamIdPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.beamIdPtr)
            return data_view.get()

        @beamIdPtr.setter
        def beamIdPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.beamIdPtr)
            data_view.set(value)

        @property
        def bufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            return data_view.get()

        @bufferSize.setter
        def bufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.bufferSize)
            data_view.set(value)

        @property
        def cudaDeviceIndex(self):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            return data_view.get()

        @cudaDeviceIndex.setter
        def cudaDeviceIndex(self, value):
            data_view = og.AttributeValueHelper(self._attributes.cudaDeviceIndex)
            data_view.set(value)

        @property
        def dataPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            return data_view.get()

        @dataPtr.setter
        def dataPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.dataPtr)
            data_view.set(value)

        @property
        def distanceBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.distanceBufferSize)
            return data_view.get()

        @distanceBufferSize.setter
        def distanceBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.distanceBufferSize)
            data_view.set(value)

        @property
        def distanceDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.distanceDataType)
            return data_view.get()

        @distanceDataType.setter
        def distanceDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.distanceDataType)
            data_view.set(value)

        @property
        def distancePtr(self):
            data_view = og.AttributeValueHelper(self._attributes.distancePtr)
            return data_view.get()

        @distancePtr.setter
        def distancePtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.distancePtr)
            data_view.set(value)

        @property
        def elevationBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.elevationBufferSize)
            return data_view.get()

        @elevationBufferSize.setter
        def elevationBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevationBufferSize)
            data_view.set(value)

        @property
        def elevationDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.elevationDataType)
            return data_view.get()

        @elevationDataType.setter
        def elevationDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevationDataType)
            data_view.set(value)

        @property
        def elevationPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.elevationPtr)
            return data_view.get()

        @elevationPtr.setter
        def elevationPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevationPtr)
            data_view.set(value)

        @property
        def emitterIdBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdBufferSize)
            return data_view.get()

        @emitterIdBufferSize.setter
        def emitterIdBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdBufferSize)
            data_view.set(value)

        @property
        def emitterIdDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdDataType)
            return data_view.get()

        @emitterIdDataType.setter
        def emitterIdDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdDataType)
            data_view.set(value)

        @property
        def emitterIdPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdPtr)
            return data_view.get()

        @emitterIdPtr.setter
        def emitterIdPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.emitterIdPtr)
            data_view.set(value)

        @property
        def exec(self):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            return data_view.get()

        @exec.setter
        def exec(self, value):
            data_view = og.AttributeValueHelper(self._attributes.exec)
            data_view.set(value)

        @property
        def height(self):
            data_view = og.AttributeValueHelper(self._attributes.height)
            return data_view.get()

        @height.setter
        def height(self, value):
            data_view = og.AttributeValueHelper(self._attributes.height)
            data_view.set(value)

        @property
        def indexBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.indexBufferSize)
            return data_view.get()

        @indexBufferSize.setter
        def indexBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.indexBufferSize)
            data_view.set(value)

        @property
        def indexDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.indexDataType)
            return data_view.get()

        @indexDataType.setter
        def indexDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.indexDataType)
            data_view.set(value)

        @property
        def indexPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.indexPtr)
            return data_view.get()

        @indexPtr.setter
        def indexPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.indexPtr)
            data_view.set(value)

        @property
        def intensityBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.intensityBufferSize)
            return data_view.get()

        @intensityBufferSize.setter
        def intensityBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensityBufferSize)
            data_view.set(value)

        @property
        def intensityDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.intensityDataType)
            return data_view.get()

        @intensityDataType.setter
        def intensityDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensityDataType)
            data_view.set(value)

        @property
        def intensityPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.intensityPtr)
            return data_view.get()

        @intensityPtr.setter
        def intensityPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensityPtr)
            data_view.set(value)

        @property
        def materialIdBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.materialIdBufferSize)
            return data_view.get()

        @materialIdBufferSize.setter
        def materialIdBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialIdBufferSize)
            data_view.set(value)

        @property
        def materialIdDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.materialIdDataType)
            return data_view.get()

        @materialIdDataType.setter
        def materialIdDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialIdDataType)
            data_view.set(value)

        @property
        def materialIdPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.materialIdPtr)
            return data_view.get()

        @materialIdPtr.setter
        def materialIdPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialIdPtr)
            data_view.set(value)

        @property
        def normalBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.normalBufferSize)
            return data_view.get()

        @normalBufferSize.setter
        def normalBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.normalBufferSize)
            data_view.set(value)

        @property
        def normalDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.normalDataType)
            return data_view.get()

        @normalDataType.setter
        def normalDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.normalDataType)
            data_view.set(value)

        @property
        def normalPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.normalPtr)
            return data_view.get()

        @normalPtr.setter
        def normalPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.normalPtr)
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
        def numReturnsPerScan(self):
            data_view = og.AttributeValueHelper(self._attributes.numReturnsPerScan)
            return data_view.get()

        @numReturnsPerScan.setter
        def numReturnsPerScan(self, value):
            data_view = og.AttributeValueHelper(self._attributes.numReturnsPerScan)
            data_view.set(value)

        @property
        def objectIdBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.objectIdBufferSize)
            return data_view.get()

        @objectIdBufferSize.setter
        def objectIdBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectIdBufferSize)
            data_view.set(value)

        @property
        def objectIdDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.objectIdDataType)
            return data_view.get()

        @objectIdDataType.setter
        def objectIdDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectIdDataType)
            data_view.set(value)

        @property
        def objectIdPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.objectIdPtr)
            return data_view.get()

        @objectIdPtr.setter
        def objectIdPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectIdPtr)
            data_view.set(value)

        @property
        def renderProductPath(self):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
            return data_view.get()

        @renderProductPath.setter
        def renderProductPath(self, value):
            data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
            data_view.set(value)

        @property
        def ticksPerScan(self):
            data_view = og.AttributeValueHelper(self._attributes.ticksPerScan)
            return data_view.get()

        @ticksPerScan.setter
        def ticksPerScan(self, value):
            data_view = og.AttributeValueHelper(self._attributes.ticksPerScan)
            data_view.set(value)

        @property
        def timestampBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.timestampBufferSize)
            return data_view.get()

        @timestampBufferSize.setter
        def timestampBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.timestampBufferSize)
            data_view.set(value)

        @property
        def timestampDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.timestampDataType)
            return data_view.get()

        @timestampDataType.setter
        def timestampDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.timestampDataType)
            data_view.set(value)

        @property
        def timestampPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.timestampPtr)
            return data_view.get()

        @timestampPtr.setter
        def timestampPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.timestampPtr)
            data_view.set(value)

        @property
        def transform(self):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            return data_view.get()

        @transform.setter
        def transform(self, value):
            data_view = og.AttributeValueHelper(self._attributes.transform)
            data_view.set(value)

        @property
        def velocityBufferSize(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityBufferSize)
            return data_view.get()

        @velocityBufferSize.setter
        def velocityBufferSize(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityBufferSize)
            data_view.set(value)

        @property
        def velocityDataType(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityDataType)
            return data_view.get()

        @velocityDataType.setter
        def velocityDataType(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityDataType)
            data_view.set(value)

        @property
        def velocityPtr(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityPtr)
            return data_view.get()

        @velocityPtr.setter
        def velocityPtr(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityPtr)
            data_view.set(value)

        @property
        def width(self):
            data_view = og.AttributeValueHelper(self._attributes.width)
            return data_view.get()

        @width.setter
        def width(self, value):
            data_view = og.AttributeValueHelper(self._attributes.width)
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
        self.inputs = OgnIsaacCreateRTXLidarScanBufferDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacCreateRTXLidarScanBufferDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacCreateRTXLidarScanBufferDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
