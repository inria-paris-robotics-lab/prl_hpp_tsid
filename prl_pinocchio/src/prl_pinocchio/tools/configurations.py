import pinocchio as pin
import rospy

class ConfigurationConvertor:
    def __init__(self, pin_model, ros_joint):
        self.q_pin_neutral = list(pin.neutral(pin_model))
        self.v_pin_neutral = [0] * pin_model.nv

        # Init variables for conversion
        self._idx_q_pin_to_ros = [-1] * len(self.q_pin_neutral) # For each component of q (for pin) give the associated idx in q (for ros). -1 means no measure associated, take from neutral q.
        self._idx_v_pin_to_ros = [-1] * len(self.v_pin_neutral) # idem
        self._idx_q_ros_to_pin = [-1] * len(ros_joint)
        self._idx_v_ros_to_pin = [-1] * len(ros_joint)
        for ros_idx, joint_name in enumerate(ros_joint):
            pin_index = pin_model.names.tolist().index(joint_name)
            pin_nq = pin_model.joints[pin_index].nq
            pin_nv = pin_model.joints[pin_index].nv
            if pin_nq == 1:
                pin_idxq = pin_model.joints[pin_index].idx_q
                self._idx_q_pin_to_ros[pin_idxq] = ros_idx
                self._idx_q_ros_to_pin[ros_idx] = pin_idxq
            if pin_nv == 1:
                pin_idxv = pin_model.joints[pin_index].idx_v
                self._idx_v_pin_to_ros[pin_idxv] = ros_idx
                self._idx_v_ros_to_pin[ros_idx] = pin_idxv

    def q_pin_to_ros(self, q_pin):
        q_res = []
        for pin_idxq in self._idx_q_ros_to_pin:
            assert pin_idxq != -1, "Cannot convert configuration : not all ros joint can be found in pin model"
            q_res.append(q_pin[pin_idxq])
        return q_res

    def q_ros_to_pin(self, q_ros):
        q_res = self.q_pin_neutral[:]
        for pin_idxq, ros_idxq in enumerate(self._idx_q_pin_to_ros):
            if(ros_idxq != -1):
                q_res[pin_idxq] = q_ros[ros_idxq]
        return q_res

    def v_pin_to_ros(self, v_pin):
        v_res = []
        for pin_idxv in self._idx_v_ros_to_pin:
            assert pin_idxv != -1, "Cannot convert configuration : not all ros joint can be found in pin model"
            v_res.append(v_pin[pin_idxv])
        return v_res

    def v_ros_to_pin(self, v_ros):
        v_res = self.v_pin_neutral[:]
        for pin_idxv, ros_idxv in enumerate(self._idx_v_pin_to_ros):
            if(ros_idxv != -1):
                v_res[pin_idxv] = v_ros[ros_idxv]
        return v_res

    ''' Return a vector of size nv, with True for pin joint corresponding to a ros joint, and False else where'''
    def v_pin_mask(self):
        return [False if idx == -1 else True for idx in self._idx_v_pin_to_ros]

    def debug_print(self):
        print("q:")
        print("pin\t->\tros")
        for pin_idxq, ros_idxq in enumerate(self._idx_q_pin_to_ros):
            print(str(pin_idxq) + "\t->\t" + str(ros_idxq))