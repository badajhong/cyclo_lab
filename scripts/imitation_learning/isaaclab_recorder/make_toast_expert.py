"""Scene-state expert: grasp, two-arm handover, gravity-assisted insertion, and lever contact."""

import torch
import isaaclab.utils.math as math_utils

from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2 import mdp, bimanual_mdp
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2.assets import SLOT_Y_BOUNDS, BREAD_BOUNDS


class ExpertFailure(RuntimeError):
    pass


class SceneExpert:
    """Use measured finger contact frames; move objects only through robot contact."""

    def __init__(self, env, progress=None, motion_speed=2.0):
        self.env = env
        self.robot = env.scene["robot"]
        self.device = env.device
        self.progress = progress or (lambda *args: None)
        self.motion_speed = motion_speed
        self.goals = {side: self.tcp_pose(side).clone() for side in ("left", "right")}
        self.home = {side: value.clone() for side, value in self.goals.items()}
        self.body_home = env.get_robot_eef_pose("body")[0].clone()
        self.opening = {"left": 1.0, "right": 1.0}
        self.lift = 0.0
        self.steps = 0
        self.milestones = []
        self.lever_start = float(mdp.lever_position(env)[0, 0])
        self.lever_min = self.lever_start
        self.lever_max = self.lever_start
        self.peak_shell_force = 0.0
        self.peak_lever_force = 0.0
        self.peak_plate_force = 0.0
        self.last_shell_force = 0.0

    def tensor(self, value):
        return torch.as_tensor(value, device=self.device, dtype=torch.float32)

    def tcp_pose(self, side):
        robot = self.robot
        letter = "l" if side == "left" else "r"
        points = []
        for suffix, sign in (("r2", 1), ("l2", -1)):
            index = robot.body_names.index(f"gripper_{letter}_rh_p12_rn_{suffix}")
            points.append(robot.data.body_link_pos_w[0, index] + math_utils.quat_apply(
                robot.data.body_link_quat_w[0, index], self.tensor([0, sign * .0045, .025]),
            ))
        wrist = self.env.get_robot_eef_pose(side)[0]
        pose = wrist.clone()
        pose[:3, 3] = (points[0] + points[1]) / 2 - self.env.scene.env_origins[0]
        return pose

    def object_pose(self, name):
        return self.env.get_object_poses()[name][0].clone()

    def frame(self, closing_axis, approach_axis):
        z = -self.tensor(approach_axis)
        z = z / torch.linalg.vector_norm(z)
        y = self.tensor(closing_axis)
        y = y - torch.dot(y, z) * z
        y = y / torch.linalg.vector_norm(y)
        x = torch.linalg.cross(y, z)
        return torch.stack((x, y, z), dim=-1)

    def target(self, position, rotation):
        return math_utils.make_pose(self.tensor(position), self.tensor(rotation))

    def move(self, label, side=None, pose=None, opening=None, steps=160, hold=False, lift=None, speed=.004,
             object_name=None, object_target=None, tolerance=.006):
        speed *= self.motion_speed
        if hold:
            steps = max(12, round(steps / self.motion_speed))
        if side is not None:
            if pose is not None:
                self.goals[side] = pose.clone()
            if opening is not None:
                self.opening[side] = float(opening)
        consecutive = 0
        error = float("inf")
        shell_force = lever_force = plate_force = 0.0
        for iteration in range(steps):
            if object_name is not None:
                current_object = self.object_pose(object_name)
                current_tcp = self.tcp_pose(side)
                if torch.linalg.vector_norm(current_object[:3, 3] - current_tcp[:3, 3]) > .10:
                    self.progress(label + "_lost_grasp", {"object": current_object.tolist(),
                                  "tcp": current_tcp.tolist(), "gripper_joints": self.robot.data.joint_pos[0, [
                                      self.robot.joint_names.index(f"gripper_l_joint{i}") for i in range(1, 5)
                                  ]].tolist()})
                    raise ExpertFailure(f"Lost grasp on {object_name} during {label}")
                # Rotate about the observed bread center in small increments. Sending the
                # final wrist pose directly can sweep the bread into the table mid-turn.
                displacement = .15 * self.motion_speed * (object_target[:3, 3] - current_object[:3, 3])
                displacement *= min(1.0, speed / max(float(torch.linalg.vector_norm(displacement)), 1e-6))
                rotation_error = math_utils.axis_angle_from_quat(math_utils.quat_from_matrix(
                    object_target[:3, :3] @ current_object[:3, :3].T,
                ))
                rotation_error *= min(1.0, .010 * self.motion_speed
                                      / max(float(torch.linalg.vector_norm(rotation_error)), 1e-6))
                increment = math_utils.matrix_from_quat(math_utils.quat_from_angle_axis(
                    torch.linalg.vector_norm(rotation_error), rotation_error,
                ))
                previous_position = self.goals[side][:3, 3].clone()
                self.goals[side] = current_tcp.clone()
                self.goals[side][:3, :3] = increment @ current_tcp[:3, :3]
                self.goals[side][:3, 3] = (previous_position + displacement
                                         + (increment - torch.eye(3, device=self.device))
                                         @ (current_tcp[:3, 3] - current_object[:3, 3]))
                tracking = self.goals[side][:3, 3] - current_tcp[:3, 3]
                self.goals[side][:3, 3] = current_tcp[:3, 3] + tracking * min(
                    1.0, .025 / max(float(torch.linalg.vector_norm(tracking)), 1e-6),
                )
            targets = {}
            for hand in ("left", "right"):
                wrist = self.env.get_robot_eef_pose(hand)[0]
                current_tcp = self.tcp_pose(hand)
                local_offset = wrist[:3, :3].T @ (current_tcp[:3, 3] - wrist[:3, 3])
                target = self.goals[hand].clone()
                target[:3, 3] -= target[:3, :3] @ local_offset
                targets[hand] = target
            if lift is None:
                mean_z = float((targets["left"][2, 3] + targets["right"][2, 3]) / 2)
                requested_lift = min(0.0, max(-.08, mean_z - 1.37))
            else:
                requested_lift = lift
            self.lift += min(.0012 * self.motion_speed,
                             max(-.0012 * self.motion_speed, requested_lift - self.lift))
            targets["body"] = self.body_home.clone()
            targets["body"][2, 3] += self.lift
            grips = {hand: self.tensor([2 * self.opening[hand] - 1]) for hand in ("left", "right")}
            grips["body"] = self.tensor([0])
            action = self.env.target_eef_pose_to_action(targets, grips)
            for offset in (0, 6):
                distance = float(torch.linalg.vector_norm(action[offset:offset + 3]) * .3)
                if distance > speed:
                    action[offset:offset + 3] *= speed / distance
                # Thin slices can slip under fast wrist rotation even when translation is slow.
                angular_step = (.025 if object_name is not None else .10) * self.motion_speed
                action[offset + 3:offset + 6].clamp_(-angular_step, angular_step)
            self.env.step(action[None])
            self.steps += 1
            lever_position = float(mdp.lever_position(self.env)[0, 0])
            self.lever_min = min(self.lever_min, lever_position)
            self.lever_max = max(self.lever_max, lever_position)
            for sensor_name in ("toaster_shell_contact", "toaster_lever_contact", "plate_contact"):
                sensor = self.env.scene.sensors.get(sensor_name)
                if sensor is not None:
                    forces = sensor.data.force_matrix_w_history
                    force = float(torch.linalg.vector_norm(forces, dim=-1).max())
                    if sensor_name == "toaster_shell_contact":
                        shell_force = max(shell_force, force)
                    elif sensor_name == "toaster_lever_contact":
                        lever_force = max(lever_force, force)
                    else:
                        plate_force = max(plate_force, force)
            self.peak_shell_force = max(self.peak_shell_force, shell_force)
            self.peak_lever_force = max(self.peak_lever_force, lever_force)
            self.peak_plate_force = max(self.peak_plate_force, plate_force)
            lever_reached = False
            if label == "lever_press":
                toaster = self.env.scene["toaster"]
                index = toaster.joint_names.index("joint_1")
                limits = toaster.data.joint_pos_limits[0, index]
                if lever_position >= float(limits[0] + .95 * (limits[1] - limits[0])):
                    lever_reached = True
            if any(part in label for part in ("present", "receive", "turn", "above_slot", "toast_",
                                              "extract", "clear_toaster", "plate")) and shell_force > 2.0:
                raise ExpertFailure(f"Robot hit toaster housing during {label}: {shell_force:.2f} N")
            if plate_force > 2.0:
                raise ExpertFailure(f"Robot hit serving plate during {label}: {plate_force:.2f} N")
            if getattr(self.env.cfg, "serve_to_plate", False):
                plate = self.env.scene["plate"]
                drift = mdp.object_position(self.env, "plate")[0, :2] - plate.data.default_root_state[0, :2]
                if torch.linalg.vector_norm(drift) > .015:
                    raise ExpertFailure(f"Serving plate moved from its arrangement during {label}")
            errors = []
            angles = []
            for hand in ("left", "right"):
                tcp = self.tcp_pose(hand)
                errors.append(float(torch.linalg.vector_norm(tcp[:3, 3] - self.goals[hand][:3, 3])))
                relative = self.goals[hand][:3, :3] @ tcp[:3, :3].T
                angles.append(float(torch.acos(((torch.trace(relative) - 1) / 2).clamp(-1, 1))))
            error = max(errors)
            if object_name is not None:
                current_object = self.object_pose(object_name)
                error = max(error, float(torch.linalg.vector_norm(current_object[:3, 3] - object_target[:3, 3])))
                relative = object_target[:3, :3] @ current_object[:3, :3].T
                angles.append(float(torch.acos(((torch.trace(relative) - 1) / 2).clamp(-1, 1))))
            if mdp.bread_dropped(self.env).any():
                raise ExpertFailure(f"Bread fell off the table during {label}")
            if not torch.isfinite(self.robot.data.joint_pos).all():
                raise ExpertFailure(f"Non-finite joint state during {label}")
            consecutive = consecutive + 1 if error < tolerance and max(angles) < .06 else 0
            if lever_reached:
                break
            if not hold and consecutive >= 3:
                break
        self.last_shell_force = shell_force
        self.progress(label, {"tcp_error_m": error, "steps": self.steps,
                              "tcp": {s: self.tcp_pose(s)[:3, 3].tolist() for s in ("left", "right")},
                              "target": {s: self.goals[s][:3, 3].tolist() for s in ("left", "right")},
                              "rotation_error_rad": angles,
                              "right_arm_joints": self.robot.data.joint_pos[0, [
                                  self.robot.joint_names.index(f"arm_r_joint{i}") for i in range(1, 8)
                              ]].tolist(),
                              "left_arm_joints": self.robot.data.joint_pos[0, [
                                  self.robot.joint_names.index(f"arm_l_joint{i}") for i in range(1, 8)
                              ]].tolist(),
                              "lift_m": float(bimanual_mdp.lift_position(self.env)[0, 0]),
                              "lever_joint_position_m": mdp.lever_position(self.env)[0].tolist(),
                              "lever_travel_m": self.lever_max - self.lever_start,
                              "shell_contact_peak_n": shell_force, "lever_contact_peak_n": lever_force,
                              "plate_contact_peak_n": plate_force,
                              "toast_cycle": bimanual_mdp.toast_cycle_state(self.env)[0].tolist(),
                              "plate_position_m": (mdp.object_position(self.env, "plate")[0].tolist()
                                                   if getattr(self.env.cfg, "serve_to_plate", False) else None),
                              "gripper_joints": {s: self.robot.data.joint_pos[0, [
                                  self.robot.joint_names.index(f"gripper_{letter}_joint{i}") for i in range(1, 5)
                              ]].tolist() for s, letter in (("left", "l"), ("right", "r"))},
                              "bread": {f"bread_{i}": mdp.object_position(self.env, f"bread_{i}")[0].tolist()
                                        for i in range(2)}})
        if not hold and (error > .025 or max(angles) > .15):
            raise ExpertFailure(f"Unreachable/colliding TCP target during {label}: {error:.3f} m")

    def mark(self, name):
        self.milestones.append({"name": name, "step": self.steps})

    def pickup_frames(self, name):
        bread = self.object_pose(name)
        normal = bread[:3, 2].clone()
        closing = normal if normal[1] >= 0 else -normal
        approach_axis = bread[:3, 0]
        rotation = self.frame(closing, approach_axis)
        contact = bread[:3, 3] + bread[:3, :3] @ self.tensor([-.035, 0, .0056])
        pickup = self.target(contact, rotation)
        above = pickup.clone()
        above[:3, 3] -= approach_axis * .10
        return pickup, above

    def pick_and_handover(self, name, bread_index, stop_after):
        initial_z = float(self.object_pose(name)[2, 3])
        pickup, above = self.pickup_frames(name)
        # Preshape to a 30 mm jaw aperture before entering the 50 mm rack pitch.
        self.move(f"{name}_approach", "right", above, .25, steps=220)
        self.move(f"{name}_descend", "right", pickup, .25, steps=150, speed=.002)
        self.move(f"{name}_close", "right", pickup, 0.0, steps=24, hold=True)
        raised = pickup.clone()
        raised[2, 3] += .12
        self.move(f"{name}_lift", "right", raised, 0.0, steps=180, speed=.002)
        if float(self.object_pose(name)[2, 3]) < initial_z + .08:
            raise ExpertFailure(f"Right gripper did not lift {name}")
        self.mark(f"right_grasp_{bread_index}")
        if stop_after == "grasp":
            return False
        grasp = torch.linalg.inv(self.tcp_pose("right")) @ self.object_pose(name)
        handover = torch.eye(4, device=self.device)
        # Keep both wrist +X axes forward during the handover; the bread may face down.
        handover[:3, :3] = self.tensor([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        handover[:3, 3] = self.tensor([.45, -.025, 1.43])
        self.move(f"{name}_present", "right", handover @ torch.linalg.inv(grasp), 0.0, steps=500, speed=.002,
                  object_name=name, object_target=handover)
        bread = self.object_pose(name)
        approach = -bread[:3, 0]
        contact = bread[:3, 3] + bread[:3, :3] @ self.tensor([.020, 0, .0056])
        receiver = self.target(contact, self.frame(bread[:3, 2], approach))
        receiver_above = receiver.clone()
        receiver_above[:3, 3] -= approach * .10
        # Both hands start above 1.42 m. Move and turn at this high level toward
        # the central handover, keeping the whole sweep above the housing.
        self.move(f"{name}_receive_approach", "left", receiver_above, .25, steps=240)
        # A 10 mm capture tolerance fits the preshaped jaws. The previous 6 mm
        # threshold stalled for seconds against wrist joint 5's limit.
        self.move(f"{name}_receive", "left", receiver, .25, steps=180, speed=.002, tolerance=.010)
        self.move(f"{name}_receive_close", "left", receiver, 0.0, steps=24, hold=True)
        self.move(f"{name}_giver_open", "right", opening=1.0, steps=36, hold=True)
        right_away = self.goals["right"].clone()
        right_away[:3, 3] += right_away[:3, 2] * .12
        self.move(f"{name}_giver_retreat", "right", right_away, 1.0, steps=150)
        if torch.linalg.vector_norm(self.object_pose(name)[:3, 3] - self.tcp_pose("left")[:3, 3]) > .09:
            raise ExpertFailure(f"Left gripper did not receive {name}")
        self.mark(f"left_receive_{bread_index}")
        if stop_after == "handover":
            return False
        self.move(f"{name}_giver_clear", "right", self.home["right"], 1.0, steps=240)
        return True

    def run(self, stop_after=None, resume_bread=None):
        if resume_bread is None:
            self.move("settle", steps=40, hold=True, lift=0)
        for bread_index in range(2):
            if resume_bread is not None and bread_index < resume_bread:
                continue
            name = f"bread_{bread_index}"
            if resume_bread is None or bread_index != resume_bread:
                if not self.pick_and_handover(name, bread_index, stop_after):
                    return False
            grasp = torch.linalg.inv(self.tcp_pose("left")) @ self.object_pose(name)
            toaster = self.object_pose("toaster")
            slot_y = sum(SLOT_Y_BOUNDS[bread_index]) / 2
            local = torch.eye(4, device=self.device)
            local[:3, :3] = self.tensor([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
            local[:3, 3] = self.tensor([-.003, slot_y, .30])
            # Finish the bread's turnover above the handover location. Translating
            # and turning simultaneously near the slots risks sweeping the shell.
            turned = self.object_pose(name)
            turned[:3, :3] = (toaster @ local)[:3, :3]
            turned[2, 3] = max(float(turned[2, 3]), 1.43)
            self.move(f"{name}_turn_upright", "left", turned @ torch.linalg.inv(grasp), 0.0,
                      steps=500, speed=.002, object_name=name, object_target=turned)
            self.move(f"{name}_above_slot", "left", toaster @ local @ torch.linalg.inv(grasp), 0.0,
                  steps=600, speed=.002, object_name=name, object_target=toaster @ local)
            # Engage the lower 15-20 mm before opening. The proximal finger links
            # are wider than the slot, so pushing the closed hand deeper hits the rim.
            local[2, 3] = .120
            self.move(f"{name}_insert", "left", toaster @ local @ torch.linalg.inv(grasp), 0.0,
                      steps=300, speed=.0015, object_name=name, object_target=toaster @ local)
            # A full 106 mm opening would sweep through the adjacent inserted slice.
            self.move(f"{name}_release", "left", opening=.25, steps=32, hold=True)
            retreat = self.goals["left"].clone()
            retreat[2, 3] += .15
            self.move(f"{name}_retreat", "left", retreat, .25, steps=180)
            if not bool(bimanual_mdp.bread_in_slot(self.env, name, bread_index)[0]):
                raise ExpertFailure(f"{name} did not settle upright in its slot")
            self.mark(f"insert_{bread_index}")
            if bread_index == 0:
                # Prepare the second pickup while the left arm leaves the
                # toaster, instead of serializing two independent motions.
                _, above = self.pickup_frames("bread_1")
                self.goals["right"] = above
                self.opening["right"] = .25
            self.move(f"{name}_left_standby", "left", self.home["left"], 1.0, steps=240)
            # The idle arm has already recovered while the other arm inserts.
            # Null-space recovery continues during the next pickup/approach;
            # avoid a fixed five-second pause after each slice.
        if resume_bread is None or resume_bread < 3:
            self.press_lever_and_return()
        if getattr(self.env.cfg, "serve_to_plate", False):
            self.move("toast_ready", steps=25, hold=True, lift=0)
            self.serve_to_plate(start_index=max(0, (resume_bread or 3) - 3))
        success = bool(bimanual_mdp.task_done(self.env)[0])
        if not success:
            raise ExpertFailure("Final physical task completion check failed")
        self.mark("return_standby")
        return True

    def press_lever_and_return(self, contact_x=.125, tool_twist=1.5707963268,
                               body_lift=None, tilt=1.0471975512):
        if bool(mdp.lever_pressed(self.env)[0]):
            raise ExpertFailure("Lever was already down before deliberate robot contact")
        # Prepare the empty hand while stationary before contacting the lever.
        self.move("lever_preshape_open", "left", self.home["left"], 1.0, steps=30, lift=0)
        # Contact the outer edge of the actual lever geometry, away from the toaster body.
        toaster = self.object_pose("toaster")
        # Tilt outward at the robot-facing edge: a vertical hand's 54 mm palm
        # overlaps the shell before the lever completes its 78 mm travel.
        lever = toaster[:3, 3] + toaster[:3, :3] @ self.tensor([contact_x, -.0005, .055])
        angle = self.tensor(tool_twist)
        zero, one = self.tensor(0.0), self.tensor(1.0)
        rotation = torch.stack((torch.stack((angle.cos(), -angle.sin(), zero)),
                                torch.stack((angle.sin(), angle.cos(), zero)), torch.stack((zero, zero, one))))
        twist = rotation
        rotation = toaster[:3, :3] @ self.tensor([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        angle = self.tensor(-tilt)
        inclination = torch.stack((torch.stack((angle.cos(), zero, angle.sin())),
                                   torch.stack((zero, one, zero)),
                                   torch.stack((-angle.sin(), zero, angle.cos()))))
        rotation = rotation @ inclination @ twist
        press = self.target(lever + self.tensor([0, 0, .05]), rotation)
        # Close while moving through free space. Check actual linkage positions
        # at the approach waypoint before permitting contact with the lever.
        self.move("lever_approach", "left", press, 0.0, steps=240, lift=body_lift)
        gripper_ids = [self.robot.joint_names.index(f"gripper_l_joint{i}") for i in range(1, 5)]
        if (self.robot.data.joint_pos[0, gripper_ids] - 1.10).abs().max() > .03:
            raise ExpertFailure("Empty left gripper did not close symmetrically before lever contact")
        press[2, 3] = lever[2] + .001
        self.move("lever_contact", "left", press, 0.0, steps=200, speed=.0015, lift=body_lift)
        press[2, 3] -= .082
        # Allow contact force through bounded IK tracking error. Stop at measured
        # 95% travel, avoiding a continued push against the mechanical end stop.
        self.move("lever_press", "left", press, 0.0, steps=160, hold=True, speed=.006, lift=body_lift)
        if (not bool(mdp.lever_pressed(self.env)[0]) or self.lever_max - self.lever_start < .065
                or self.peak_lever_force < .1):
            raise ExpertFailure("Robot contact did not depress the lever")
        if self.last_shell_force > 2.0:
            raise ExpertFailure(f"Lever press hit toaster housing: {self.last_shell_force:.2f} N")
        self.mark("press_lever")
        # Clear the outer lever edge before raising the hand. Use the measured
        # pose because the press intentionally ends at measured lever travel.
        press = self.tcp_pose("left")
        press[:3, 3] += toaster[:3, 0] * .025 + self.tensor([0, 0, .025])
        self.move("lever_clear", "left", press, 0.0, steps=100)
        press[2, 3] += .18
        self.move("lever_retreat", "left", press, 0.0, steps=220)
        self.move("left_standby", "left", self.home["left"], 1.0, steps=260, lift=0)
        self.move("right_standby", "right", self.home["right"], 1.0, steps=260, lift=0)
        self.move("final", steps=25, hold=True, lift=0)
        if self.env.toast_cycle is not None:
            for _ in range(10):
                if self.env.toast_cycle.phase[0] == 4:
                    break
                self.move("toast_wait", steps=25, hold=True, lift=0)
            if self.env.toast_cycle.phase[0] != 4:
                raise ExpertFailure("Toaster did not finish its five-second heating/pop-up cycle")
            self.mark("toast_pop_up")

    def serve_to_plate(self, start_index=0):
        """Extract from above, carry clear of the rim, and release near the plate."""
        import itertools
        if self.env.toast_cycle.phase[0] != 4:
            raise ExpertFailure("Serving cannot start before the toaster has popped up")
        for i in range(start_index, 2):
            name = f"bread_{i}"
            bread = self.object_pose(name)
            initial_z = float(bread[2, 3])
            # The measured finger TCP lies along wrist -Z (not wrist +X).
            # Tilt its +Z upward toward the robot, keeping the wrist in reach
            # and the palm above the housing during the exposed-top pinch.
            up = bread[:3, 1]
            closing = bread[:3, 2]
            along = bread[:3, 0]
            tool_z = .8660254 * up + .5 * along
            rotation = torch.stack((torch.linalg.cross(closing, tool_z), closing, tool_z), dim=-1)
            contact = bread[:3, 3] + bread[:3, :3] @ self.tensor([0, .036, .0056])
            pickup = self.target(contact, rotation)
            above = pickup.clone()
            above[:3, 3] += tool_z * .07
            self.move(f"{name}_toast_approach", "left", above, .15, steps=320)
            self.move(f"{name}_toast_contact", "left", pickup, .15, steps=180, speed=.0015)
            self.move(f"{name}_toast_close", "left", pickup, 0.0, steps=30, hold=True)
            extracted = pickup.clone()
            extracted[2, 3] += .17
            self.move(f"{name}_extract", "left", extracted, 0.0, steps=200, speed=.0025)
            if float(self.object_pose(name)[2, 3]) < initial_z + .12:
                raise ExpertFailure(f"Left gripper did not extract {name} from the toaster")
            if self.last_shell_force > 2:
                raise ExpertFailure(f"Extraction hit toaster housing: {self.last_shell_force:.2f} N")
            self.mark(f"extract_toast_{i}")
            grasp = torch.linalg.inv(self.tcp_pose("left")) @ self.object_pose(name)
            carried = self.object_pose(name)
            plate = self.object_pose("plate")
            carried[:3, 3] = self.tensor([float(plate[0, 3]), float(plate[1, 3]), 1.32])
            self.move(f"{name}_clear_toaster", "left", carried @ torch.linalg.inv(grasp), 0.0,
                      steps=400, speed=.002, object_name=name, object_target=carried)
            # A 30 degree inclination clears the plate with the lower finger.
            # Fully opening the large jaws needs 50 mm clearance above the support.
            # gravity then lays it flat, with no attachment or pose writes.
            angle = self.tensor(torch.pi / 6)
            c, s = angle.cos(), angle.sin()
            z, o = self.tensor(0), self.tensor(1)
            carried[:3, :3] = torch.stack((torch.stack((o, z, z)),
                                           torch.stack((z, c, -s)), torch.stack((z, s, c))))
            carried[:3, :3] = self.tensor([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) @ carried[:3, :3]
            carried[2, 3] = 1.30
            self.move(f"{name}_turn_to_plate", "left", carried @ torch.linalg.inv(grasp), 0.0,
                      steps=600, speed=.002, object_name=name, object_target=carried)
            plate = self.object_pose("plate")
            support = float(plate[2, 3]) + .0178308 * .22
            xy = plate[:2, 3].clone()
            corners = self.tensor(list(itertools.product(*BREAD_BOUNDS)))
            if i:
                previous = self.object_pose("bread_0")
                support = max(support, float((corners @ previous[:3, :3].T + previous[:3, 3])[:, 2].max()))
                xy[0] -= .003
            carried[:2, 3] = xy + self.tensor([-.004, -.003])
            lower_extent = float((corners @ carried[:3, :3].T)[:, 2].min())
            carried[2, 3] = support - lower_extent + .050
            self.move(f"{name}_lower_to_plate", "left", carried @ torch.linalg.inv(grasp), 0.0,
                      steps=400, speed=.0015, object_name=name, object_target=carried)
            self.move(f"{name}_plate_release", "left", opening=1.0, steps=60, hold=True)
            grip_ids = [self.robot.joint_names.index(f"gripper_l_joint{j}") for j in range(1, 5)]
            if self.robot.data.joint_pos[0, grip_ids].abs().max() > .15:
                raise ExpertFailure(f"Left fingers did not fully release {name} before withdrawal")
            for _ in range(6):
                if bool(bimanual_mdp.bread_on_plate(self.env, name)[0]):
                    break
                self.move(f"{name}_drop_settle", steps=25, hold=True)
            if not bool(bimanual_mdp.bread_on_plate(self.env, name)[0]):
                self.progress(f"{name}_release_check_failed", {
                    "bread_pose": self.object_pose(name).tolist(), "plate_pose": self.object_pose("plate").tolist(),
                    "bread_linear_velocity_mps": self.env.scene[name].data.root_lin_vel_w[0].tolist(),
                })
                raise ExpertFailure(f"{name} was not free and settled on the plate before withdrawal")
            retreat = self.goals["left"].clone()
            retreat[2, 3] += .12
            self.move(f"{name}_plate_retreat", "left", retreat, 1.0, steps=160)
            for _ in range(6):
                self.move(f"{name}_plate_settle", steps=25, hold=True)
                if bool(bimanual_mdp.bread_on_plate(self.env, name)[0]):
                    break
            if not bool(bimanual_mdp.bread_on_plate(self.env, name)[0]):
                self.progress(f"{name}_plate_check_failed", {
                    "bread_pose": self.object_pose(name).tolist(), "plate_pose": self.object_pose("plate").tolist(),
                    "bread_linear_velocity_mps": self.env.scene[name].data.root_lin_vel_w[0].tolist(),
                })
                raise ExpertFailure(f"{name} did not settle flat within the serving plate")
            self.mark(f"serve_toast_{i}")
            self.move(f"{name}_serve_standby", "left", self.home["left"], 1.0, steps=300, lift=0)
        for _ in range(12):
            self.move("served_final", steps=25, hold=True, lift=0)
            if bool(bimanual_mdp.task_done(self.env)[0]):
                break
