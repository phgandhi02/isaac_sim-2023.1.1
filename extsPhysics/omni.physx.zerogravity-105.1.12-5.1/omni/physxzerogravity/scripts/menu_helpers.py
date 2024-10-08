import asyncio
from abc import ABC, abstractmethod

import carb.settings
import omni.ui as ui

from .constants import ColliderTypes
from .settings import ZeroGravitySettings
from .styles import Styles


class PopupMenu(ABC):
    POPUP_DELAY = 0.3

    def __init__(self) -> None:
        self._popup_menu = ui.Menu("", style={"Separator": {"color": Styles.SEPARATOR_COLOR}})
        self._popup_task = None

    def __del__(self):
        self._popup_menu.hide()
        self._popup_menu.clear()
        self._popup_menu = None

    @abstractmethod
    def _build_menu(self):
        raise NotImplementedError("Derived class must implement this method!")

    def hide(self):
        self.cancel_popup_task()
        self._popup_menu.hide()

    async def _show_popup_delayed(self, x, y):
        await asyncio.sleep(self.POPUP_DELAY)
        self._show_popup(x, y)

    def _show_popup(self, x, y):
        self._popup_menu.clear()
        with self._popup_menu:
            self._build_menu()
        self._popup_menu.show()
        if self._popup_task is not None:
            self._popup_task = None

    def show_popup(self, x, y, delayed: bool):
        if delayed:
            self._popup_task = asyncio.ensure_future(
                self._show_popup_delayed(x, y))
        else:
            self._show_popup(x, y)

    def cancel_popup_task(self):
        if self._popup_task:
            self._popup_task.cancel()


class MenuHelpers:
    # static members
    _static_coll_approximations = {
        ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_TRIANGLE_MESH: "Triangle Mesh",
        ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_MESH_SIMPLIFICATION: "Mesh Simplification",
    }
    _dynamic_coll_approximations = {
        ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_HULL: "Convex Hull",
        ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_DECOMPOSITION: "Convex Decomposition",
    }

    @staticmethod
    def create_settings_menu_item_bool(caption: str, setting: str, enabled: bool = True, hide_on_click: bool = True):
        settings = carb.settings.get_settings()
        ui.MenuItem(
            caption,
            enabled=enabled,
            checkable=True,
            hide_on_click=hide_on_click,
            checked=settings.get_as_bool(setting),
            checked_changed_fn=lambda enabled: settings.set_bool(setting, enabled)
        )

    @staticmethod
    def create_settings_menu_item_int(caption: str, setting: str, val: int, enabled: bool = True, hide_on_click: bool = True):
        settings = carb.settings.get_settings()
        ui.MenuItem(
            caption,
            enabled=enabled,
            checkable=True,
            hide_on_click=hide_on_click,
            checked=settings.get_as_int(setting) == val,
            checked_changed_fn=lambda _, v=val: settings.set_int(setting, v),
        )

    @staticmethod
    def create_static_coll_menu():
        ui.Separator('Static Collider Approximation'),
        for k, v in MenuHelpers._static_coll_approximations.items():
            MenuHelpers.create_settings_menu_item_int(v, ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, k)

    @staticmethod
    def create_dynamic_coll_menu():
        ui.Separator('Dynamic Collider Approximation'),
        for k, v in MenuHelpers._dynamic_coll_approximations.items():
            MenuHelpers.create_settings_menu_item_int(v, ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, k)
