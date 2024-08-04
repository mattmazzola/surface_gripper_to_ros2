import asyncio
import gc
import weakref

import omni
import omni.kit.app
import omni.kit.commands
import omni.log
import omni.timeline
import omni.ui as ui
import omni.ui.workspace_utils
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_TITLE, MENU_BAR_BUTTON_NAME
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        name = EXTENSION_TITLE
        # Build Window
        self._window = ScrollingWindow(
            title=name,
            width=600,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)
        self._menu_items = [
            make_menu_item_description(self.ext_id, name, lambda a=weakref.proxy(self): a._menu_callback())
        ]

        add_menu_items(self._menu_items, MENU_BAR_BUTTON_NAME)

        self.ui_builder = UIBuilder()
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        omni.log.info(f"on_shutdown")
        remove_menu_items(self._menu_items, MENU_BAR_BUTTON_NAME)

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(target_window, docking_window_title, location, ratio=0.5):
                docking_window = omni.ui.Workspace.get_window(docking_window_title)
                if docking_window and target_window:
                    docking_window.dock_in(target_window, location, ratio)
                return docking_window

            target_window_title = "Viewport"
            target_window = ui.Workspace.get_window(target_window_title)
            dock(target_window, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        self.ui_builder.on_timeline_event(event)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        self.ui_builder.build_ui()
