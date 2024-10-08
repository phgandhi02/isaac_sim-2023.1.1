import omni.kit.app


def is_kit_vp2() -> bool:
    """returns True is current app is running Kit ViewPort v2, otherwise returns False."""
    import omni.kit.viewport.utility
    active_vp_window = omni.kit.viewport.utility.get_active_viewport_window()
    return not hasattr(active_vp_window, "legacy_window")


def get_rigid_body_parent_prim(prim):
    from pxr import UsdPhysics
    parent = prim
    while (parent and not parent.IsPseudoRoot()):
        if parent.HasAPI(UsdPhysics.RigidBodyAPI):
            return parent
        parent = parent.GetParent()
    return None


async def ui_wait(updates_cnt=10):
    for _ in range(updates_cnt):
        await omni.kit.app.get_app().next_update_async()


def refresh_manipulator_selector():
    if is_kit_vp2():
        # activate appropriate manipulator
        from omni.kit.manipulator.selector import get_manipulator_selector
        get_manipulator_selector("")._refresh()


def get_physics_physx_schemas_applied(prim):
    '''returns applied physics and physx schemas'''
    from pxr import Usd, Plug
    tokens = []
    schemas = prim.GetAppliedSchemas()
    for schema in schemas:
        schema_split = schema.split(":")
        schemaToken = Usd.SchemaRegistry().GetTypeFromSchemaTypeName(schema_split[0])
        physics = Plug.Registry().GetPluginWithName("usdPhysics")
        physx = Plug.Registry().GetPluginWithName("physxSchema")
        if (physx and physx.DeclaresType(schemaToken)) or (physics and physics.DeclaresType(schemaToken)):
            tokens.append(schemaToken)
    return tokens
