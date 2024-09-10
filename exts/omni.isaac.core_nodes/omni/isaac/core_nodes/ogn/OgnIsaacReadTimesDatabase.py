"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadTimes

Read time related value from Fabric.
"""

import carb

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacReadTimesDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadTimes

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.gpu
            inputs.renderResults
        Outputs:
            outputs.durationDenominator
            outputs.durationNumerator
            outputs.execOut
            outputs.externalTimeOfSimNs
            outputs.frameNumber
            outputs.rationalTimeOfSimDenominator
            outputs.rationalTimeOfSimNumerator
            outputs.sampleTimeOffsetInSimFrames
            outputs.simulationTime
            outputs.simulationTimeMonotonic
            outputs.swhFrameNumber
            outputs.systemTime

    Predefined Tokens:
        tokens.IsaacFabricTimes
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
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:gpu', 'uint64', 0, None, 'Pointer to shared context containing gpu foundations valid only in the postRender graph.', {}, True, 0, False, ''),
        ('inputs:renderResults', 'uint64', 0, None, 'Render results', {}, True, 0, False, ''),
        ('outputs:durationDenominator', 'uint64', 0, 'Duration Denominator', 'durationDenominator', {}, True, None, False, ''),
        ('outputs:durationNumerator', 'int64', 0, 'Duration Numerator', 'durationNumerator', {}, True, None, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'The output execution port', {}, True, None, False, ''),
        ('outputs:externalTimeOfSimNs', 'int64', 0, 'External Time Of Sim Ns', 'externalTimeOfSimNs', {}, True, None, False, ''),
        ('outputs:frameNumber', 'int64', 0, 'Frame Number', 'frameNumber', {}, True, None, False, ''),
        ('outputs:rationalTimeOfSimDenominator', 'uint64', 0, 'Rational Time Of Sim Denominator', 'rationalTimeOfSimDenominator', {}, True, None, False, ''),
        ('outputs:rationalTimeOfSimNumerator', 'int64', 0, 'Rational Time Of Sim Numerator', 'rationalTimeOfSimNumerator', {}, True, None, False, ''),
        ('outputs:sampleTimeOffsetInSimFrames', 'uint64', 0, 'Sample Time Offset In Sim Frames', 'sampleTimeOffsetInSimFrames', {}, True, None, False, ''),
        ('outputs:simulationTime', 'double', 0, 'Simulation Time', 'Current Simulation Time in Seconds', {}, True, None, False, ''),
        ('outputs:simulationTimeMonotonic', 'double', 0, 'Monotonic Simulation Time', 'Current Monotonic Simulation Time in Seconds', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'int64', 0, 'swhFrameNumber', 'swhFrameNumber', {}, True, None, False, ''),
        ('outputs:systemTime', 'double', 0, 'System Time', 'Current System Time in Seconds', {}, True, None, False, ''),
    ])

    class tokens:
        IsaacFabricTimes = "IsaacFabricTimes"

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
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
        def gpu(self):
            data_view = og.AttributeValueHelper(self._attributes.gpu)
            return data_view.get()

        @gpu.setter
        def gpu(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.gpu)
            data_view = og.AttributeValueHelper(self._attributes.gpu)
            data_view.set(value)

        @property
        def renderResults(self):
            data_view = og.AttributeValueHelper(self._attributes.renderResults)
            return data_view.get()

        @renderResults.setter
        def renderResults(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.renderResults)
            data_view = og.AttributeValueHelper(self._attributes.renderResults)
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
        def durationDenominator(self):
            data_view = og.AttributeValueHelper(self._attributes.durationDenominator)
            return data_view.get()

        @durationDenominator.setter
        def durationDenominator(self, value):
            data_view = og.AttributeValueHelper(self._attributes.durationDenominator)
            data_view.set(value)

        @property
        def durationNumerator(self):
            data_view = og.AttributeValueHelper(self._attributes.durationNumerator)
            return data_view.get()

        @durationNumerator.setter
        def durationNumerator(self, value):
            data_view = og.AttributeValueHelper(self._attributes.durationNumerator)
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
        def externalTimeOfSimNs(self):
            data_view = og.AttributeValueHelper(self._attributes.externalTimeOfSimNs)
            return data_view.get()

        @externalTimeOfSimNs.setter
        def externalTimeOfSimNs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.externalTimeOfSimNs)
            data_view.set(value)

        @property
        def frameNumber(self):
            data_view = og.AttributeValueHelper(self._attributes.frameNumber)
            return data_view.get()

        @frameNumber.setter
        def frameNumber(self, value):
            data_view = og.AttributeValueHelper(self._attributes.frameNumber)
            data_view.set(value)

        @property
        def rationalTimeOfSimDenominator(self):
            data_view = og.AttributeValueHelper(self._attributes.rationalTimeOfSimDenominator)
            return data_view.get()

        @rationalTimeOfSimDenominator.setter
        def rationalTimeOfSimDenominator(self, value):
            data_view = og.AttributeValueHelper(self._attributes.rationalTimeOfSimDenominator)
            data_view.set(value)

        @property
        def rationalTimeOfSimNumerator(self):
            data_view = og.AttributeValueHelper(self._attributes.rationalTimeOfSimNumerator)
            return data_view.get()

        @rationalTimeOfSimNumerator.setter
        def rationalTimeOfSimNumerator(self, value):
            data_view = og.AttributeValueHelper(self._attributes.rationalTimeOfSimNumerator)
            data_view.set(value)

        @property
        def sampleTimeOffsetInSimFrames(self):
            data_view = og.AttributeValueHelper(self._attributes.sampleTimeOffsetInSimFrames)
            return data_view.get()

        @sampleTimeOffsetInSimFrames.setter
        def sampleTimeOffsetInSimFrames(self, value):
            data_view = og.AttributeValueHelper(self._attributes.sampleTimeOffsetInSimFrames)
            data_view.set(value)

        @property
        def simulationTime(self):
            data_view = og.AttributeValueHelper(self._attributes.simulationTime)
            return data_view.get()

        @simulationTime.setter
        def simulationTime(self, value):
            data_view = og.AttributeValueHelper(self._attributes.simulationTime)
            data_view.set(value)

        @property
        def simulationTimeMonotonic(self):
            data_view = og.AttributeValueHelper(self._attributes.simulationTimeMonotonic)
            return data_view.get()

        @simulationTimeMonotonic.setter
        def simulationTimeMonotonic(self, value):
            data_view = og.AttributeValueHelper(self._attributes.simulationTimeMonotonic)
            data_view.set(value)

        @property
        def swhFrameNumber(self):
            data_view = og.AttributeValueHelper(self._attributes.swhFrameNumber)
            return data_view.get()

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            data_view = og.AttributeValueHelper(self._attributes.swhFrameNumber)
            data_view.set(value)

        @property
        def systemTime(self):
            data_view = og.AttributeValueHelper(self._attributes.systemTime)
            return data_view.get()

        @systemTime.setter
        def systemTime(self, value):
            data_view = og.AttributeValueHelper(self._attributes.systemTime)
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
        self.inputs = OgnIsaacReadTimesDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadTimesDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadTimesDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
