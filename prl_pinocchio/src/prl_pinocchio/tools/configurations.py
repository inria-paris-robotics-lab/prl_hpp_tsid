import pinocchio as pin

class ConfigurationConvertor:
    def __init__(self, pin_model, ros_joint):
        self.q_pin_neutral = list(pin.neutral(pin_model))
        self.v_pin_neutral = [0] * pin_model.nv

        # Init ros to pin
        self._idx_q_pin_to_ros = [-1] * len(self.q_pin_neutral) # For each component of q (for pin) give the associated idx in q (for ros). -1 means no measure associated, take from neutral q.
        self._idx_v_pin_to_ros = [-1] * len(self.v_pin_neutral) # idem
        for ros_idx, joint_name in enumerate(ros_joint):
            pin_index = pin_model.names.tolist().index(joint_name)
            pin_nq = pin_model.joints[pin_index].nq
            pin_nv = pin_model.joints[pin_index].nv
            if pin_nq == 1:
                pin_idxq = pin_model.joints[pin_index].idx_q
                self._idx_q_pin_to_ros[pin_idxq] = ros_idx
            if pin_nv == 1:
                pin_idxv = pin_model.joints[pin_index].idx_v
                self._idx_v_pin_to_ros[pin_idxv] = ros_idx

    def q_pin_to_ros(self, q_pin):
        pass

    def q_ros_to_pin(self, q_ros):
        q_res = self.q_pin_neutral[:]
        for pin_idxq, ros_idxq in enumerate(self._idx_q_pin_to_ros):
            if(ros_idxq != -1):
                q_res[pin_idxq] = q_ros[ros_idxq]
        return q_res

    def v_pin_to_ros(self, v):
        pass

    def v_ros_to_pin(self, v_ros):
        v_res = self.v_pin_neutral[:]
        for pin_idxv, ros_idxv in enumerate(self._idx_v_pin_to_ros):
            if(ros_idxv != -1):
                v_res[pin_idxv] = v_ros[ros_idxv]
        return v_res