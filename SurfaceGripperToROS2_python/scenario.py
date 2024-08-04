from abc import ABC

import numpy as np
import omni.kit.commands
import omni.log
import omni.timeline
from omni.isaac.core.objects.cone import DynamicCone
from omni.isaac.core.objects.cuboid import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.stage import (
    create_new_stage,
    get_current_stage,
)
from omni.isaac.core.utils.viewports import set_camera_view
from pxr import Sdf, UsdLux


class ScenarioBase(ABC):
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def on_physics_step(self):
        pass


class SurfaceGripperToRos2Scenario(ScenarioBase):
    def __init__(self):
        omni.log.info("__init__")
        super().__init__()

        self.total_time = 0.0
        self.color_netural = [1, 1, 1]
        self.color_blue = [0.0, 0.0, 1.0]
        self.color_red = [1.0, 0.0, 0.0]
        self.color_green = [0.0, 1.0, 0.0]

    def init_scenario(self):
        omni.log.info("init_scenario")
        create_new_stage()

    async def setup_scenario(self):
        omni.log.info("setup_scenario")

        GroundPlane("/World/ground_plane")

        stage = get_current_stage()
        dome_light: UsdLux.DomeLight = UsdLux.DomeLight.Define(stage, "/World/dome_light")
        dome_light.CreateIntensityAttr(1000)

        cube = DynamicCuboid("/World/cube", position=[0, 0, 0.5])
        cube.prim.GetAttribute("primvars:displayColor").Set(np.array(self.color_blue))
        # Remove the default material assigned so that primvars:displayColor can be seen
        omni.kit.commands.execute(
            "BindMaterial",
            material_path=None,
            prim_path=[cube.prim_path],
            strength=["weakerThanDescendants"],
            material_purpose="",
        )

        cone = DynamicCone("/World/cone", position=[0, 0, 1.5], scale=[0.5, 0.5, 1])
        cone.prim.GetAttribute("primvars:displayColor").Set(np.array(self.color_netural))
        # Remove the default material assigned so that primvars:displayColor can be seen
        omni.kit.commands.execute(
            "BindMaterial",
            material_path=None,
            prim_path=[cone.prim_path],
            strength=["weakerThanDescendants"],
            material_purpose="",
        )

        # Set the refinement level of the cone to improve the visual quality
        cone.prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool)
        cone.prim.GetAttribute("refinementEnableOverride").Set(True)
        cone.prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int)
        cone.prim.GetAttribute("refinementLevel").Set(2)

        set_camera_view(
            eye=[3.0, 4.3, 3.2],
            target=[0, 0, 1],
            camera_prim_path="/OmniverseKit_Persp",
        )

    def teardown_scenario(self):
        omni.log.info("teardown_scenario")

    def on_physics_step(self, step_duration: float):
        if not omni.timeline.get_timeline_interface().is_playing():
            return

        self.total_time += step_duration
        omni.log.info(f"update_scenario: step_duration: {step_duration}")
        omni.log.info(f"update_scenario: total_time: {self.total_time}")
