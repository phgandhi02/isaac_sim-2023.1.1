"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.DifferentialController

Differential Controller
"""

import numpy
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnDifferentialControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.DifferentialController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.angularVelocity
            inputs.execIn
            inputs.linearVelocity
            inputs.maxAngularSpeed
            inputs.maxLinearSpeed
            inputs.maxWheelSpeed
            inputs.wheelDistance
            inputs.wheelRadius
        Outputs:
            outputs.effortCommand
            outputs.positionCommand
            outputs.velocityCommand
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
        ('inputs:angularVelocity', 'double', 0, 'Angular Velocity', 'desired rotation velocity', {}, True, 0.0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:linearVelocity', 'double', 0, 'Linear Velocity', 'desired linear velocity', {}, True, 0.0, False, ''),
        ('inputs:maxAngularSpeed', 'double', 0, None, 'max angular speed allowed for vehicle', {}, True, 0.0, False, ''),
        ('inputs:maxLinearSpeed', 'double', 0, None, 'max linear speed allowed for vehicle', {}, True, 0.0, False, ''),
        ('inputs:maxWheelSpeed', 'double', 0, None, 'max wheel speed allowed', {}, True, 0.0, False, ''),
        ('inputs:wheelDistance', 'double', 0, None, 'distance between the two wheels', {}, True, 0.0, False, ''),
        ('inputs:wheelRadius', 'double', 0, None, 'radius of the wheels', {}, True, 0.0, False, ''),
        ('outputs:effortCommand', 'double[]', 0, None, 'effort commands', {}, True, None, False, ''),
        ('outputs:positionCommand', 'double[]', 0, None, 'position commands', {}, True, None, False, ''),
        ('outputs:velocityCommand', 'double[]', 0, None, 'velocity commands', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"angularVelocity", "execIn", "linearVelocity", "maxAngularSpeed", "maxLinearSpeed", "maxWheelSpeed", "wheelDistance", "wheelRadius", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.angularVelocity, self._attributes.execIn, self._attributes.linearVelocity, self._attributes.maxAngularSpeed, self._attributes.maxLinearSpeed, self._attributes.maxWheelSpeed, self._attributes.wheelDistance, self._attributes.wheelRadius]
            self._batchedReadValues = [0.0, None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        @property
        def angularVelocity(self):
            return self._batchedReadValues[0]

        @angularVelocity.setter
        def angularVelocity(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def linearVelocity(self):
            return self._batchedReadValues[2]

        @linearVelocity.setter
        def linearVelocity(self, value):
            self._batchedReadValues[2] = value

        @property
        def maxAngularSpeed(self):
            return self._batchedReadValues[3]

        @maxAngularSpeed.setter
        def maxAngularSpeed(self, value):
            self._batchedReadValues[3] = value

        @property
        def maxLinearSpeed(self):
            return self._batchedReadValues[4]

        @maxLinearSpeed.setter
        def maxLinearSpeed(self, value):
            self._batchedReadValues[4] = value

        @property
        def maxWheelSpeed(self):
            return self._batchedReadValues[5]

        @maxWheelSpeed.setter
        def maxWheelSpeed(self, value):
            self._batchedReadValues[5] = value

        @property
        def wheelDistance(self):
            return self._batchedReadValues[6]

        @wheelDistance.setter
        def wheelDistance(self, value):
            self._batchedReadValues[6] = value

        @property
        def wheelRadius(self):
            return self._batchedReadValues[7]

        @wheelRadius.setter
        def wheelRadius(self, value):
            self._batchedReadValues[7] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

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
            self.effortCommand_size = None
            self.positionCommand_size = None
            self.velocityCommand_size = None
            self._batchedWriteValues = { }

        @property
        def effortCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            return data_view.get(reserved_element_count=self.effortCommand_size)

        @effortCommand.setter
        def effortCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            data_view.set(value)
            self.effortCommand_size = data_view.get_array_size()

        @property
        def positionCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            return data_view.get(reserved_element_count=self.positionCommand_size)

        @positionCommand.setter
        def positionCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            data_view.set(value)
            self.positionCommand_size = data_view.get_array_size()

        @property
        def velocityCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            return data_view.get(reserved_element_count=self.velocityCommand_size)

        @velocityCommand.setter
        def velocityCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            data_view.set(value)
            self.velocityCommand_size = data_view.get_array_size()

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
        self.inputs = OgnDifferentialControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnDifferentialControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnDifferentialControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.DifferentialController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnDifferentialControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnDifferentialControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnDifferentialControllerDatabase(node)

            try:
                compute_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnDifferentialControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnDifferentialControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)

            per_node_data = OgnDifferentialControllerDatabase.PER_NODE_DATA[node.node_id()]
            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnDifferentialControllerDatabase._release_per_node_data(node)

        @staticmethod
        def release_instance(node, target):
            OgnDifferentialControllerDatabase._release_per_node_instance_data(node, target)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Differential Controller")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot controller inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Differential Controller")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnDifferentialControllerDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnDifferentialControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnDifferentialControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnDifferentialControllerDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.DifferentialController")
