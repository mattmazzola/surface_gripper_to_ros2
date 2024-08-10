from abc import ABC

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.log
import omni.timeline
from omni.isaac.core.objects import DynamicCone, DynamicCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import (
    create_new_stage,
    get_current_stage,
    get_next_free_path,
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

        # Create Prim for Gripper Force Origin
        gripper_force_origin_prim = XFormPrim(
            prim_path=get_next_free_path("gripper_force_origin", cone.prim_path),
            name="gripper_force_origin",
            # Manually add small offset to improve calculation
            # Warning: Surface Gripper is inside the parent Rigid body collider. please move it forward in the X offset direction by 0.001000 to avoid wasted computation
            translation=[0, 0, -0.5 - 0.001],
            orientation=euler_angles_to_quat([0, 90, 0], degrees=True),
        )

        keys = og.Controller.Keys

        og_path = get_next_free_path("SurfaceGripperActionGraph", "/graph")
        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    # Inputs
                    ("OnImpulseClose", "omni.graph.action.OnImpulseEvent"),
                    ("OnImpulseOpen", "omni.graph.action.OnImpulseEvent"),
                    ("OnKeyboardInputClose", "omni.graph.action.OnKeyboardInput"),
                    ("OnKeyboardInputOpen", "omni.graph.action.OnKeyboardInput"),
                    # Ticks
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SurfaceGripperNode", "omni.isaac.surface_gripper.SurfaceGripper"),
                    # ROS
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2Publisher", "omni.isaac.ros2_bridge.ROS2Publisher"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("IsaacTimeSplitter", "omni.isaac.core_nodes.IsaacTimeSplitter"),
                    # Position Selection
                    ("ConstantDoubleOpen", "omni.graph.nodes.ConstantDouble"),
                    ("ConstantDoubleClose", "omni.graph.nodes.ConstantDouble"),
                    ("SelectIfPosition", "omni.graph.nodes.SelectIf"),
                    ("ConstructArrayPosition", "omni.graph.nodes.ConstructArray"),
                    # Color Selection
                    ("ConstantColor3fOpen", "omni.graph.nodes.ConstantColor3f"),
                    ("ConstantColor3fClose", "omni.graph.nodes.ConstantColor3f"),
                    ("SelectIfColor", "omni.graph.nodes.SelectIf"),
                    ("ConstructArrayColor", "omni.graph.nodes.ConstructArray"),
                    # Gripper Color
                    ("WriteGripperColor", "omni.graph.nodes.WritePrimAttribute"),
                    # Gripper State Debug
                    ("ConstantStringOpen", "omni.graph.nodes.ConstantString"),
                    ("ConstantStringClose", "omni.graph.nodes.ConstantString"),
                    ("SelectIfGripperState", "omni.graph.nodes.SelectIf"),
                    ("PrintGripperState", "omni.graph.ui_nodes.PrintText"),
                ],
                keys.SET_VALUES: [
                    # Inputs
                    ("OnImpulseClose.inputs:onlyPlayback", False),
                    ("OnImpulseOpen.inputs:onlyPlayback", False),
                    ("OnKeyboardInputClose.inputs:keyIn", "C"),
                    ("OnKeyboardInputClose.inputs:onlyPlayback", False),
                    ("OnKeyboardInputOpen.inputs:keyIn", "O"),
                    ("OnKeyboardInputOpen.inputs:onlyPlayback", False),
                    ("SurfaceGripperNode.inputs:ParentRigidBody", cone.prim_path),
                    ("SurfaceGripperNode.inputs:GripPosition", gripper_force_origin_prim.prim_path),
                    # ROS
                    ("IsaacReadSimulationTime.inputs:resetOnStop", False),
                    ("ROS2Publisher.inputs:messagePackage", "sensor_msgs"),
                    ("ROS2Publisher.inputs:messageName", "JointState"),
                    ("ROS2Publisher.inputs:nodeNamespace", "surface_gripper"),
                    ("ROS2Publisher.inputs:topicName", "joint_states"),
                    # Position Selection
                    ("ConstantDoubleOpen.inputs:value", 0.0),
                    ("ConstantDoubleClose.inputs:value", 1.0),
                    # Color Selection
                    ("ConstantColor3fOpen.inputs:value", self.color_green),
                    ("ConstantColor3fClose.inputs:value", self.color_red),
                    # Gripper Color
                    ("WriteGripperColor.inputs:prim", cone.prim_path),
                    # Gripper State Debug
                    ("ConstantStringOpen.inputs:value", "Open"),
                    ("ConstantStringClose.inputs:value", "Close"),
                    ("PrintGripperState.inputs:toScreen", True),
                ],
                keys.CONNECT: [
                    # Inputs
                    ("OnImpulseClose.outputs:execOut", "SurfaceGripperNode.inputs:Close"),
                    ("OnImpulseOpen.outputs:execOut", "SurfaceGripperNode.inputs:Open"),
                    ("OnKeyboardInputClose.outputs:pressed", "SurfaceGripperNode.inputs:Close"),
                    ("OnKeyboardInputOpen.outputs:pressed", "SurfaceGripperNode.inputs:Open"),
                    # Ticks
                    ("OnPlaybackTick.outputs:tick", "WriteGripperColor.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ROS2Publisher.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PrintGripperState.inputs:execIn"),
                    # ROS
                    ("ROS2Context.outputs:context", "ROS2Publisher.inputs:context"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "IsaacTimeSplitter.inputs:time"),
                    # Cannot connect until ROS2Publisher message type is set
                    # ("IsaacTimeSplitter.outputs:seconds", "ROS2Publisher.inputs:header:stamp:sec"),
                    # ("IsaacTimeSplitter.outputs:nanoseconds", "ROS2Publisher.inputs:header:stamp:nanosec"),
                    # Position Selection
                    ("ConstantDoubleOpen.inputs:value", "SelectIfPosition.inputs:ifFalse"),
                    ("ConstantDoubleClose.inputs:value", "SelectIfPosition.inputs:ifTrue"),
                    ("SurfaceGripperNode.outputs:Closed", "SelectIfPosition.inputs:condition"),
                    ("SelectIfPosition.outputs:result", "ConstructArrayPosition.inputs:input0"),
                    # Cannot connect until ROS2Publisher message type is set
                    # ("ConstructArrayPosition.outputs:array", "ROS2Publisher.inputs:position"),
                    # Color Selection
                    ("ConstantColor3fOpen.inputs:value", "SelectIfColor.inputs:ifFalse"),
                    ("ConstantColor3fClose.inputs:value", "SelectIfColor.inputs:ifTrue"),
                    ("SurfaceGripperNode.outputs:Closed", "SelectIfColor.inputs:condition"),
                    ("SelectIfColor.outputs:result", "ConstructArrayColor.inputs:input0"),
                    ("ConstructArrayColor.outputs:array", "WriteGripperColor.inputs:value"),
                    # Gripper State Debug
                    ("ConstantStringOpen.inputs:value", "SelectIfGripperState.inputs:ifFalse"),
                    ("ConstantStringClose.inputs:value", "SelectIfGripperState.inputs:ifTrue"),
                    ("SurfaceGripperNode.outputs:Closed", "SelectIfGripperState.inputs:condition"),
                    ("SelectIfGripperState.outputs:result", "PrintGripperState.inputs:text"),
                ],
            },
        )

        # Note: Certain properties cannot be set until other pre-requisite properties are applied which change the state of node
        # We set properties in 2 stages to avoid this issue
        og_path = graph_handle.get_path_to_graph()
        og.Controller.set(
            og.Controller.attribute(og_path + "/WriteGripperColor.inputs:name"),
            "primvars:displayColor",
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/IsaacTimeSplitter.outputs:seconds"),
            og.Controller.attribute(og_path + "/ROS2Publisher.inputs:header:stamp:sec"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/IsaacTimeSplitter.outputs:nanoseconds"),
            og.Controller.attribute(og_path + "/ROS2Publisher.inputs:header:stamp:nanosec"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/ConstructArrayPosition.outputs:array"),
            og.Controller.attribute(og_path + "/ROS2Publisher.inputs:position"),
        )

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
        omni.log.verbose(f"update_scenario: step_duration: {step_duration}")
        omni.log.verbose(f"update_scenario: total_time: {self.total_time}")

    def close_gripper(self):
        if omni.timeline.get_timeline_interface().is_playing():
            og.Controller.set(
                og.Controller.attribute("/graph/SurfaceGripperActionGraph/OnImpulseClose.state:enableImpulse"),
                True,
            )
            omni.log.info(f"close_gripper")

    def open_gripper(self):
        if omni.timeline.get_timeline_interface().is_playing():
            og.Controller.set(
                og.Controller.attribute("/graph/SurfaceGripperActionGraph/OnImpulseOpen.state:enableImpulse"),
                True,
            )
            omni.log.info(f"open_gripper")
