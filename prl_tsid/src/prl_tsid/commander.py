import tsid
import rospy
import numpy as np
import pinocchio as pin

class PathFollower:
    TIMEOUT_STEP = 2
    COLLISION_STEP = 2
    PRIO_COST = 1
    PRIO_CONSTRAINT = 0

    def __init__(self, robot):
        ## Init the problem
        self.robot = robot
        self.tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, tsid.FIXED_BASE_SYSTEM, False)
        self.formulation = tsid.InverseDynamicsFormulationAccForce("tsid", self.tsid_robot, False)

        ## Some default values
        q0 = np.zeros(self.tsid_robot.nq)
        v0 = np.zeros(self.tsid_robot.nv)
        a0 = np.zeros(self.tsid_robot.na)

        self.formulation.computeProblemData(0.0, q0, v0)

        # Joint torque bounds task
        self.w_actuationBounds = 1.0
        self.actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", self.tsid_robot)
        self.formulation.addActuationTask(self.actuationBoundsTask, self.w_actuationBounds, self.PRIO_CONSTRAINT, 0.0)

        # Joint pos, vel, acc bounds task
        self.w_posVelAccBounds = 1.0
        self.jointBoundsTask = tsid.TaskJointPosVelAccBounds("task-joint-bounds", self.tsid_robot, 0.1) # dt will be re-set before executing
        self.jointBoundsTask.setVerbose(False)
        self.formulation.addMotionTask(self.jointBoundsTask, self.w_posVelAccBounds, self.PRIO_CONSTRAINT, 0.0)

        ## Init the tasks
        self._set_position_limit()
        self.set_velocity_limit(1)
        self.set_acceleration_limit(1)
        self.set_torque_limit(1)

        ## Create the solver
        self.solver = tsid.SolverProxQP("qp solver")
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)

    def _set_position_limit(self):
        self.jointBoundsTask.setPositionBounds(self.tsid_robot.model().lowerPositionLimit,
                                               self.tsid_robot.model().upperPositionLimit)

    def set_velocity_limit(self, scale):
        v_max = scale * self.tsid_robot.model().velocityLimit
        self.jointBoundsTask.setVelocityBounds(v_max)

    def set_acceleration_limit(self, scale):
        a_max = scale * self.robot.MAX_JOINT_ACC * np.ones(self.tsid_robot.nv)
        self.jointBoundsTask.setAccelerationBounds(a_max)

    def set_torque_limit(self, scale):
        tau_max = scale * self.tsid_robot.model().effortLimit
        tau_min = - tau_max
        self.actuationBoundsTask.setBounds(tau_min, tau_max)

    def execute_path(self, path, commanders, dt, velocity_ctrl=False):
        # Gains
        Kp_posture = 1.
        Kd_posture = 2.0 * np.sqrt(Kp_posture)
        w_posture = 1
        Kp_ee = 1.
        w_ee = 0.1

        # Posture task
        postureTask = tsid.TaskJointPosture("task-posture", self.tsid_robot)
        postureSample = tsid.TrajectorySample(self.tsid_robot.nq, self.tsid_robot.nv)
        self.formulation.addMotionTask(postureTask, w_posture, self.PRIO_COST, 0.0)
        postureTask.setKp(Kp_posture * np.ones(self.tsid_robot.na))
        postureTask.setKd(Kd_posture * np.ones(self.tsid_robot.na))

        # Init end effector tasks
        eeTasks_names = []
        eeTasks = []
        eeSamples = []
        eeIndexes = []
        for targetFrame in path.targetFrames:
            eeIndex = self.robot.pin_robot_wrapper.model.getFrameId(targetFrame)
            if eeIndex >= len(self.robot.pin_robot_wrapper.model.frames):
                rospy.logwarn("Frame " + str(targetFrame) + " not found in the robot model : task related to that frame will be ignored")
                continue

            eeTask_name = "ee-task-" + targetFrame
            eeTask = tsid.TaskSE3Equality(eeTask_name , self.tsid_robot, targetFrame)
            eeTask.setKp(Kp_ee* np.ones(self.tsid_robot.na))
            eeTask.setKd(2.0 * np.sqrt(Kp_ee) * np.ones(self.tsid_robot.na))
            eeTask.useLocalFrame(True) # Represent jacobian in local frame

            eeIndexes.append(eeIndex)
            eeTasks_names.append(eeTask_name)
            eeTasks.append(eeTask)
            eeSamples.append(tsid.TrajectorySample(12, 6))

            self.formulation.addMotionTask(eeTask, w_ee, self.PRIO_COST, 0.0)

        self.eeTasks = eeTasks

        # Resize solver
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)

        # Update tasks parameters
        self.jointBoundsTask.setTimeStep(dt)

        # Prepare the loop
        rate = rospy.Rate(1. / dt)
        elapsed_time = 0.0
        start_time = None

        # Prepare a filter function
        def _rearrange_hpp_to_pin(q, v, v_dot):
                # Assume that the robot configuration is at first in the overall configuration
                return np.array(q[:self.robot.pin_robot_wrapper.model.nq]), \
                       np.array(v[:self.robot.pin_robot_wrapper.model.nv]), \
                       np.array(v_dot[:self.robot.pin_robot_wrapper.model.nv])

        # Initialize measures
        t, q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)
        q_meas, v_meas = np.array(q_meas), np.array(v_meas)
        v_next = np.zeros(self.tsid_robot.nv)

        # Loop
        while elapsed_time < 2.0 * path.corbaPath.length() and not rospy.is_shutdown():
            # Get robot state
            t_ros, q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)

            # Measure the current time
            if start_time is None:
                start_time = t_ros
            else:
                elapsed_time = t_ros - start_time

            # clip the elapsed time to the path length
            t = min(elapsed_time, path.corbaPath.length())

            # Get references from trajectory
            q     = path.corbaPath.call(t)[0]
            v     = path.corbaPath.derivative(t, 1)
            v_dot = path.corbaPath.derivative(t, 2)

            q, v, v_dot = _rearrange_hpp_to_pin(q,v,v_dot)

            # Set posture reference
            postureSample.value(q)
            postureSample.derivative(v)
            postureSample.second_derivative(v_dot)
            postureTask.setReference(postureSample)

            # Compute forward kinematics
            self.robot.pin_robot_wrapper.forwardKinematics(q, v, v_dot)
            for i in range(len(eeTasks)):
                ee_pos = self.robot.pin_robot_wrapper.framePlacement   (q,           eeIndexes[i], update_kinematics=False) # No need to recompute the forwad kinematic
                ee_vel = self.robot.pin_robot_wrapper.frameVelocity    (q, v,        eeIndexes[i], update_kinematics=False, reference_frame=pin.ReferenceFrame.LOCAL) # No need to recompute the forwad kinematic
                ee_acc = self.robot.pin_robot_wrapper.frameAcceleration(q, v, v_dot, eeIndexes[i], update_kinematics=False, reference_frame=pin.ReferenceFrame.LOCAL) # No need to recompute the forwad kinematic

                ee_pos_vec = np.concatenate((ee_pos.translation, ee_pos.rotation.flatten('F')))
                ee_vel_vec = ee_vel.vector
                ee_acc_vec = ee_acc.vector

                eeSamples[i].value(ee_pos_vec)
                eeSamples[i].derivative(ee_vel_vec)
                eeSamples[i].second_derivative(ee_acc_vec)

                eeTasks[i].setReference(eeSamples[i])

            # Feedback
            q_meas = np.array(q_meas)
            v_meas = np.array(v_meas)

            # Because we control the robot on velocity, a velocity feedback would destabilised the control,
            # thus we supposed that the velocity is tracked perfectly
            if(velocity_ctrl):
                v_meas = v_next

            # Solve
            HQPData = self.formulation.computeProblemData(t, q_meas, v_meas)
            sol = self.solver.solve(HQPData)
            if(sol.status!=0):
                np.set_printoptions(precision=4, suppress=True, threshold=np.inf, linewidth=np.inf)
                print("Time  " + str(t) + " QP problem could not be solved! Error code: " + str(sol.status))
                print("Posture tasks : ")
                for i in range(len(eeTasks)):
                    print("- eeIndex " + str(eeIndexes[i]))
                    print("   eeTasks_names " + str(eeTasks_names[i]))
                    print("   ee_pos_vec " + str(eeSamples[i].value()))
                    print("   ee_pos_err " + str(eeTasks[i].position_error))
                    print("   ee_vel_vec " + str(eeSamples[i].derivative()))
                    print("   ee_vel_err " + str(eeTasks[i].velocity_error))
                    print("   ee_acc_vec " + str(eeSamples[i].second_derivative()))
                break

            dv_next = self.formulation.getAccelerations(sol)
            tau_next = self.formulation.getActuatorForces(sol)

            # numerical integration
            v_next = v_meas + dt*dv_next
            v_mean = (v_next + v_meas) / 2
            q_next = pin.integrate(self.robot.pin_robot_wrapper.model, q_meas, v_mean * dt)

            # Compute collisions
            q_col, v_col, dv_col = q_next, v_next, dv_next
            col_res, col_pairs = False, []
            for i in range(self.COLLISION_STEP):
                col_res, col_pairs = self.robot.compute_collisions(q_col, True)
                if col_res:
                    break
                # integrate
                if(velocity_ctrl):
                    v_col = v_col
                    v_col_mean = v_col
                else:
                    v_col_mean = v_col + 0.5 * dt*dv_col
                    v_col = v_col + dt*dv_col
                q_col = pin.integrate(self.robot.pin_robot_wrapper.model, q_col, v_col_mean * dt)

            timeout = 0 if col_res else self.TIMEOUT_STEP*dt # Make the controller timeout instantly if a collision can occur

            # publish commands
            for commander in commanders:
                commander.execute_fwd(q_next, v_next, tau_next, timeout)

            if(col_res):
                rospy.logerr("Possible collision detected at time %f, %d steps in the future (dt = %f):\n\tCollision pairs:\n\t\t- %s\n\t\t- %s", t, i, dt, col_pairs[0][0], col_pairs[0][1])
                break

            # Wait for next step
            rate.sleep()

        # Remove all the end effector tasks and resize solver
        for taskname in eeTasks_names:
            self.formulation.removeTask(taskname, 0.0)
        self.formulation.removeTask("task-posture", 0.0)
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)


    def follow_velocity(self, targetFrame, commanders, dt, velocity_ctrl=False):
        # Gains
        Kp_ee = 0.1
        Kd_ee = 2 * np.sqrt(Kp_ee)
        w_ee = 0.1

        # Init end effector tasks
        eeIndex = self.robot.pin_robot_wrapper.model.getFrameId(targetFrame)
        assert eeIndex < len(self.robot.pin_robot_wrapper.model.frames), "Frame " + str(targetFrame) + " not found in the robot model."

        #    Follow velocity
        eeVelTask_name = "ee-vel-task-" + targetFrame
        eeVelTask = tsid.TaskSE3Equality(eeVelTask_name , self.tsid_robot, targetFrame)
        eeVelTask.useLocalFrame(True) # Represent jacobian in local frame
        eeVelTask.setKp(       np.zeros(6))
        eeVelTask.setKd(Kd_ee * np.ones(6))
        self.eeVelSample = tsid.TrajectorySample(12, 6)
        self.formulation.addMotionTask(eeVelTask, w_ee, self.PRIO_COST, 0.0)

        # Resize solver
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)

        # Update tasks parameters
        self.jointBoundsTask.setTimeStep(dt)

        # Prepare the loop
        rate = rospy.Rate(1. / dt)
        elapsed_time = 0.0
        start_time = None

        # Initialize measures
        t, q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)
        q_meas, v_meas= np.array(q_meas), np.array(v_meas)
        v_next = np.zeros(self.tsid_robot.nv)

        # Set ee referece
        self.robot.pin_robot_wrapper.forwardKinematics(q_meas, v_meas)
        ee_pos = self.robot.pin_robot_wrapper.framePlacement(q_meas, eeIndex, update_kinematics=False) # No need to recompute the forwad kinematic
        ee_pos_vec = np.concatenate((ee_pos.translation, ee_pos.rotation.flatten('F')))

        self.eeVelSample.value(ee_pos_vec)
        self.eeVelSample.derivative(np.zeros(6))
        self.eeVelSample.second_derivative(np.zeros(6))

        # Loop
        while not rospy.is_shutdown():
            # Get robot state
            t_ros, q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)

            # Measure the current time
            if start_time is None:
                start_time = t_ros
            else:
                elapsed_time = t_ros - start_time

            t = elapsed_time

            # Compute forward kinematics
            eeVelTask.setReference(self.eeVelSample)

            # Feedback
            q_meas = np.array(q_meas)
            v_meas = np.array(v_meas)

            # Because we control the robot on velocity, a velocity feedback would destabilised the control,
            # thus we supposed that the velocity is tracked perfectly
            if(velocity_ctrl):
                v_meas = v_next

            # Solve
            HQPData = self.formulation.computeProblemData(t, q_meas, v_meas)
            sol = self.solver.solve(HQPData)
            if(sol.status!=0):
                np.set_printoptions(precision=4, suppress=True, threshold=np.inf, linewidth=np.inf)
                print("Time  " + str(t) + " QP problem could not be solved! Error code: " + str(sol.status))
                print("Posture tasks : ")
                print("   eeIndex     " + str(eeIndex))
                # print("   eePosTask_name " + str(eePosTask_name))
                # print("   ee_pos_vec  " + str(self.eePosSample.value()))
                # print("   ee_pos_err  " + str(eePosTask.position_error))
                # print("   ee_vel_vec  " + str(self.eePosSample.derivative()))
                # print("   ee_vel_err  " + str(eePosTask.velocity_error))
                # print("   ee_acc_vec  " + str(self.eePosSample.second_derivative()))
                break

            dv_next = self.formulation.getAccelerations(sol)
            tau_next = self.formulation.getActuatorForces(sol)

            # numerical integration
            v_next = v_meas + dt*dv_next
            v_mean = (v_next + v_meas) / 2
            q_next = pin.integrate(self.robot.pin_robot_wrapper.model, q_meas, v_mean * dt)

            # Compute collisions
            q_col, v_col, dv_col = q_next, v_next, dv_next
            col_res, col_pairs = False, []
            for i in range(self.COLLISION_STEP):
                col_res, col_pairs = self.robot.compute_collisions(q_col, True)
                if col_res:
                    break
                # integrate
                if(velocity_ctrl):
                    v_col = v_col
                    v_col_mean = v_col
                else:
                    v_col_mean = v_col + 0.5 * dt*dv_col
                    v_col = v_col + dt*dv_col
                q_col = pin.integrate(self.robot.pin_robot_wrapper.model, q_col, v_col_mean * dt)

            timeout = 0 if col_res else self.TIMEOUT_STEP*dt # Make the controller timeout instantly if a collision can occur

            # publish commands
            for commander in commanders:
                commander.execute_fwd(q_next, v_next, tau_next, timeout)

            if(col_res):
                rospy.logerr("Possible collision detected at time %f, %d steps in the future (dt = %f):\n\tCollision pairs:\n\t\t- %s\n\t\t- %s", t, i, dt, col_pairs[0][0], col_pairs[0][1])
                break

            # Wait for next step
            rate.sleep()

        # Remove all the end effector tasks and resize solver
        self.formulation.removeTask(eeVelTask_name, 0.0)
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)