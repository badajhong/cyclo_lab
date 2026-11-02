"""Keep toast extraction vertical and compensate each measured grasp for serving."""

import torch

from isaaclab_mimic.datagen.data_generator import DataGenerator


class MakeToastServingDataGenerator(DataGenerator):
    def generate_eef_subtask_trajectory(self, env_id, eef_name, subtask_ind,
                                       all_randomized_subtask_boundaries,
                                       runtime_subtask_constraints_dict, selected_src_demo_inds):
        trajectory = super().generate_eef_subtask_trajectory(
            env_id, eef_name, subtask_ind, all_randomized_subtask_boundaries,
            runtime_subtask_constraints_dict, selected_src_demo_inds,
        )
        if eef_name != "left" or subtask_ind == 0:
            return trajectory
        demo_index = selected_src_demo_inds[eef_name]
        source = self.src_demo_datagen_info_pool.datagen_infos[demo_index]
        start, end = all_randomized_subtask_boundaries[eef_name][demo_index, subtask_ind]
        targets = source.target_eef_pose[eef_name][start:end]
        grips = source.gripper_action[eef_name][start:end].reshape(-1)
        if len(trajectory) != len(targets):
            raise ValueError("Serving retargeting requires aligned source subtask frames")
        if subtask_ind in (1, 3):
            # The action is normalized: -1 closes, +1 opens. The 15% opening
            # used for slot approach is -0.7 and must not count as a grasp.
            closed = torch.nonzero(grips <= -.99).flatten()
            if not len(closed):
                raise ValueError("A toast extraction segment contains no closed grasp")
            anchor = int(closed[0])
            position = trajectory[anchor].pose[:3, 3].clone()
            # Rotate the approach and pinch with the bread. Once held, lift
            # along WORLD Z: rotating the upward path with a tilted slice
            # sweeps the fingers into the adjacent slot or toaster rim.
            for index in range(anchor, len(trajectory)):
                pose = trajectory[index].pose.clone()
                pose[:3, 3] = position + targets[index, :3, 3] - targets[anchor, :3, 3]
                trajectory[index].pose = pose
        elif subtask_ind in (2, 4):
            name = f"bread_{0 if subtask_ind == 2 else 1}"
            current_eef = self.env.get_robot_eef_pose("left", [env_id])[0]
            current_bread = self.env.get_object_poses([env_id])[name][0]
            actual_grasp = torch.linalg.inv(current_eef) @ current_bread
            inverse_grasp = torch.linalg.inv(actual_grasp)
            correction = torch.eye(4, device=self.env.device)
            standby = torch.tensor([.3609426, .3954084, 1.4419125], device=self.env.device)
            opened = False
            for index in range(len(trajectory)):
                opened |= bool(grips[index] > -.99)
                if not opened:
                    frame = start + index
                    source_grasp = (torch.linalg.inv(source.eef_pose[eef_name][frame])
                                    @ source.object_poses[name][frame])
                    correction = source_grasp @ inverse_grasp
                # Keep the final grasp correction through opening/withdrawal;
                # never chase the bread after it is released under gravity.
                pose = trajectory[index].pose @ correction
                # A different pinch point changes the finger-opening sweep.
                # Add 20 mm only near the plate, so bread cannot drag the dish
                # before the large jaws have fully opened. Fade it away while
                # returning to standby, retaining the original final pose.
                height = float(source.object_poses[name][start + index, 2, 3])
                near_plate = max(0.0, min(1.0, (1.25 - height) / .10))
                away_from_home = min(1.0, float(torch.linalg.vector_norm(
                    trajectory[index].pose[:3, 3] - standby)) / .15)
                # Calibrated opening/flattening drift is toward the robot.
                # Center the held slice 20 mm farther onto the dish so that
                # both settled slices retain room when the second lands.
                pose[0, 3] += .020 * near_plate * away_from_home
                pose[2, 3] += .020 * near_plate * away_from_home
                trajectory[index].pose = pose
        return trajectory
