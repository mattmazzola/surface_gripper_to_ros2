import asyncio

import omni.log
import omni.timeline
import omni.ui as ui
from omni.isaac.ui.element_wrappers import CollapsableFrame, StateButton
from omni.isaac.ui.element_wrappers.core_connectors import LoadButton, ResetButton
from omni.isaac.ui.ui_utils import get_style
from omni.usd import StageEventType

from .scenario import SurfaceGripperToRos2Scenario


class UIBuilder:

    def __init__(self):
        self.wrapped_ui_elements = []
        self._timeline = omni.timeline.get_timeline_interface()
        self._on_init()

    def _on_init(self):
        self._scenario = SurfaceGripperToRos2Scenario()

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._run_stop_btn.reset()
            self._run_stop_btn.enabled = False
            self._gripper_state_btn.enabled = False

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            self._reset_extension()

    def cleanup(self):
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton(
                    "Load Button",
                    "LOAD",
                    setup_scene_fn=self._on_setup_scene,
                    setup_post_load_fn=self._on_setup_post_load,
                )
                self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario", collapsed=False)

        with run_scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._run_stop_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_click_run,
                    on_b_click_fn=self._on_click_stop,
                    physics_callback_fn=self._on_physics_step,
                )
                self._run_stop_btn.enabled = False
                self.wrapped_ui_elements.append(self._run_stop_btn)

        robot_actions_frame = CollapsableFrame("Robot Actions", collapsed=False)

        with robot_actions_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._gripper_state_btn = StateButton(
                    "Gripper State",
                    "Close",
                    "Open",
                    on_a_click_fn=self._on_click_gripper_close,
                    on_b_click_fn=self._on_click_gripper_open,
                )
                self._gripper_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._gripper_state_btn)

    def _on_setup_scene(self):
        self._scenario.init_scenario()

    def _on_setup_post_load(self):
        self._reset_scenario()

        # UI management
        self._run_stop_btn.reset()
        self._run_stop_btn.enabled = True
        self._reset_btn.enabled = True

    def _reset_scenario(self):
        omni.log.info(f"_reset_scenario")
        self._scenario.teardown_scenario()
        asyncio.ensure_future(self._scenario.setup_scenario())

    def _reset_extension(self):
        omni.log.info(f"_reset_extension")
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._run_stop_btn.reset()
        self._run_stop_btn.enabled = False
        self._gripper_state_btn.enabled = False
        self._reset_btn.enabled = False

    def _on_post_reset_btn(self):
        self._reset_scenario()

        # UI management
        self._run_stop_btn.reset()
        self._run_stop_btn.enabled = True
        self._gripper_state_btn.enabled = False

    def _on_physics_step(self, step_duration: float):
        self._scenario.on_physics_step(step_duration)

    def _on_click_run(self):
        self._gripper_state_btn.enabled = True
        self._timeline.play()

    def _on_click_stop(self):
        self._timeline.pause()

    def _on_click_gripper_close(self):
        self._scenario.close_gripper()

    def _on_click_gripper_open(self):
        self._scenario.open_gripper()
