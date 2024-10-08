"""Support for simplified access to data on nodes of type omni.physx.cct.OgnCharacterController

Activate or deactivate a Character Controller on a Capsule prim
"""

import sys
import traceback

import carb
import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnCharacterControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.physx.cct.OgnCharacterController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.activate
            inputs.capsulePath
            inputs.controlsSettings
            inputs.deactivate
            inputs.fpCameraPathToken
            inputs.gravity
            inputs.setupControls
            inputs.speed
        Outputs:
            outputs.done

    Predefined Tokens:
        tokens.Auto
        tokens.Manual
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 41, 3)
    TARGET_VERSION = (2, 139, 8)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:activate', 'execution', 0, None, 'Activate Character Controller on a Capsule. This can be done on e.g. Simulation Start Play event.', {}, True, None, False, ''),
        ('inputs:capsulePath', 'path', 0, 'Capsule Path', 'Connect a path of a capsule to use as a character controller. Use Spawn Capsule node to dynamically spawn a capsule for you if needed.', {}, True, "", False, ''),
        ('inputs:controlsSettings', 'bundle', 0, None, 'Use Controls Settings to rebind controls.', {}, False, None, False, ''),
        ('inputs:deactivate', 'execution', 0, None, 'Deactivate Character Controller on a Capsule. This can be done on e.g. Simulation Stop Play event.', {}, True, None, False, ''),
        ('inputs:fpCameraPathToken', 'token', 0, 'First Person Camera Path', 'If a camera path is connected the character controller with use first person camera mode', {}, False, None, False, ''),
        ('inputs:gravity', 'bool', 0, 'Enable Gravity', 'Enable Gravity', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:setupControls', 'token', 0, 'Setup Controls', 'Setup controls: Auto will use default WASD/mouse/gamepad controls or Controls Settings keybinds if connected. Manual will skip control setup completely, leaving it to the user to do manually.', {ogn.MetadataKeys.ALLOWED_TOKENS: 'Auto,Manual', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["Auto", "Manual"]', ogn.MetadataKeys.DEFAULT: '"Auto"'}, True, "Auto", False, ''),
        ('inputs:speed', 'int', 0, 'Speed', 'Speed in units/s', {ogn.MetadataKeys.DEFAULT: '500'}, True, 500, False, ''),
        ('outputs:done', 'execution', 0, 'Done', 'The output execution', {}, True, None, False, ''),
    ])

    class tokens:
        Auto = "Auto"
        Manual = "Manual"

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.activate = og.AttributeRole.EXECUTION
        role_data.inputs.capsulePath = og.AttributeRole.PATH
        role_data.inputs.controlsSettings = og.AttributeRole.BUNDLE
        role_data.inputs.deactivate = og.AttributeRole.EXECUTION
        role_data.outputs.done = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"activate", "capsulePath", "deactivate", "fpCameraPathToken", "gravity", "setupControls", "speed", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.activate, self._attributes.capsulePath, self._attributes.deactivate, self._attributes.fpCameraPathToken, self._attributes.gravity, self._attributes.setupControls, self._attributes.speed]
            self._batchedReadValues = [None, "", None, None, True, "Auto", 500]

        @property
        def controlsSettings(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.controlsSettings"""
            return self.__bundles.controlsSettings

        @property
        def activate(self):
            return self._batchedReadValues[0]

        @activate.setter
        def activate(self, value):
            self._batchedReadValues[0] = value

        @property
        def capsulePath(self):
            return self._batchedReadValues[1]

        @capsulePath.setter
        def capsulePath(self, value):
            self._batchedReadValues[1] = value

        @property
        def deactivate(self):
            return self._batchedReadValues[2]

        @deactivate.setter
        def deactivate(self, value):
            self._batchedReadValues[2] = value

        @property
        def fpCameraPathToken(self):
            return self._batchedReadValues[3]

        @fpCameraPathToken.setter
        def fpCameraPathToken(self, value):
            self._batchedReadValues[3] = value

        @property
        def gravity(self):
            return self._batchedReadValues[4]

        @gravity.setter
        def gravity(self, value):
            self._batchedReadValues[4] = value

        @property
        def setupControls(self):
            return self._batchedReadValues[5]

        @setupControls.setter
        def setupControls(self, value):
            self._batchedReadValues[5] = value

        @property
        def speed(self):
            return self._batchedReadValues[6]

        @speed.setter
        def speed(self, value):
            self._batchedReadValues[6] = value

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
        LOCAL_PROPERTY_NAMES = {"done", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def done(self):
            value = self._batchedWriteValues.get(self._attributes.done)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.done)
                return data_view.get()

        @done.setter
        def done(self, value):
            self._batchedWriteValues[self._attributes.done] = value

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
        self.inputs = OgnCharacterControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnCharacterControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnCharacterControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.physx.cct.OgnCharacterController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnCharacterControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnCharacterControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnCharacterControllerDatabase(node)

            try:
                compute_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnCharacterControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnCharacterControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)

            per_node_data = OgnCharacterControllerDatabase.PER_NODE_DATA[node.node_id()]
            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnCharacterControllerDatabase._release_per_node_data(node)

        @staticmethod
        def release_instance(node, target):
            OgnCharacterControllerDatabase._release_per_node_instance_data(node, target)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.physx.cct")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Physx Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Activate or deactivate a Character Controller on a Capsule prim")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(og.eAccessLocation.E_GLOBAL, og.eAccessType.E_WRITE)
                    __hints.set_data_access(og.eAccessLocation.E_USD, og.eAccessType.E_READ_WRITE)
                OgnCharacterControllerDatabase.INTERFACE.add_to_node_type(node_type)
                node_type.set_has_state(True)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnCharacterControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnCharacterControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnCharacterControllerDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.physx.cct.OgnCharacterController")
