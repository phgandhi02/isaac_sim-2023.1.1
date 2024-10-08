"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacReadIMU

Node that reads out IMU linear acceleration, angular velocity and orientation data
"""

import carb
import numpy
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacReadIMUDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacReadIMU

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.imuPrim
            inputs.readGravity
            inputs.useLatestData
        Outputs:
            outputs.angVel
            outputs.execOut
            outputs.linAcc
            outputs.orientation
            outputs.sensorTime
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
        ('inputs:imuPrim', 'target', 0, 'IMU Prim', 'Usd prim reference to the IMU prim', {}, True, [], False, ''),
        ('inputs:readGravity', 'bool', 0, None, 'True to read gravitational acceleration in the measurement, False to ignore gravitational acceleration', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:useLatestData', 'bool', 0, None, 'True to use the latest data from the physics step, False to use the data measured by the sensor', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('outputs:angVel', 'vector3d', 0, 'Angular Velocity Vector', 'Angular velocity IMU reading', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when sensor has data', {}, True, None, False, ''),
        ('outputs:linAcc', 'vector3d', 0, 'Linear Acceleration Vector', 'Linear acceleration IMU reading', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('outputs:orientation', 'quatd', 0, 'Sensor Orientation Quaternion', 'Sensor orientation as quaternion', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('outputs:sensorTime', 'float', 0, None, 'Timestamp of the sensor reading', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.imuPrim = og.AttributeRole.TARGET
        role_data.outputs.angVel = og.AttributeRole.VECTOR
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        role_data.outputs.linAcc = og.AttributeRole.VECTOR
        role_data.outputs.orientation = og.AttributeRole.QUATERNION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "readGravity", "useLatestData", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn, self._attributes.readGravity, self._attributes.useLatestData]
            self._batchedReadValues = [None, True, False]

        @property
        def imuPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.imuPrim)
            return data_view.get()

        @imuPrim.setter
        def imuPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.imuPrim)
            data_view = og.AttributeValueHelper(self._attributes.imuPrim)
            data_view.set(value)
            self.imuPrim_size = data_view.get_array_size()

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        @property
        def readGravity(self):
            return self._batchedReadValues[1]

        @readGravity.setter
        def readGravity(self, value):
            self._batchedReadValues[1] = value

        @property
        def useLatestData(self):
            return self._batchedReadValues[2]

        @useLatestData.setter
        def useLatestData(self, value):
            self._batchedReadValues[2] = value

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
        LOCAL_PROPERTY_NAMES = {"angVel", "execOut", "linAcc", "orientation", "sensorTime", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def angVel(self):
            value = self._batchedWriteValues.get(self._attributes.angVel)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.angVel)
                return data_view.get()

        @angVel.setter
        def angVel(self, value):
            self._batchedWriteValues[self._attributes.angVel] = value

        @property
        def execOut(self):
            value = self._batchedWriteValues.get(self._attributes.execOut)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.execOut)
                return data_view.get()

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        @property
        def linAcc(self):
            value = self._batchedWriteValues.get(self._attributes.linAcc)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.linAcc)
                return data_view.get()

        @linAcc.setter
        def linAcc(self, value):
            self._batchedWriteValues[self._attributes.linAcc] = value

        @property
        def orientation(self):
            value = self._batchedWriteValues.get(self._attributes.orientation)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.orientation)
                return data_view.get()

        @orientation.setter
        def orientation(self, value):
            self._batchedWriteValues[self._attributes.orientation] = value

        @property
        def sensorTime(self):
            value = self._batchedWriteValues.get(self._attributes.sensorTime)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.sensorTime)
                return data_view.get()

        @sensorTime.setter
        def sensorTime(self, value):
            self._batchedWriteValues[self._attributes.sensorTime] = value

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
        self.inputs = OgnIsaacReadIMUDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadIMUDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadIMUDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.sensor.IsaacReadIMU'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnIsaacReadIMUDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacReadIMUDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnIsaacReadIMUDatabase(node)

            try:
                compute_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnIsaacReadIMUDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)

            per_node_data = OgnIsaacReadIMUDatabase.PER_NODE_DATA[node.node_id()]
            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnIsaacReadIMUDatabase._release_per_node_data(node)

        @staticmethod
        def release_instance(node, target):
            OgnIsaacReadIMUDatabase._release_per_node_instance_data(node, target)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.sensor")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Isaac Read IMU Node")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSensor")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Node that reads out IMU linear acceleration, angular velocity and orientation data")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.sensor}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.sensor.IsaacReadIMU.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnIsaacReadIMUDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnIsaacReadIMUDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacReadIMUDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.sensor.IsaacReadIMU")
