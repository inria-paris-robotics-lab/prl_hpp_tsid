import tsid
import rospy
import numpy as np
import pinocchio as pin

class PathFollower:
    def __init__(self, robot):
        ## Init the problem
        self.robot = robot
        self.tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, True, False)
        self.formulation = tsid.InverseDynamicsFormulationAccForce("tsid", self.tsid_robot, False)

        ## Some default values
        q0 = np.zeros(self.tsid_robot.nq)
        v0 = np.zeros(self.tsid_robot.nv)
        a0 = np.zeros(self.tsid_robot.na)

        self.formulation.computeProblemData(0.0, q0, v0)

        ## Create a vizualizer
        self.robot.create_visualizer()
        self.robot.display(q0)

        ## Create the tasks
        na_ones = np.ones(self.tsid_robot.na)
        K_ee = 100
        self.w_ee = 20
        K_posture = 1
        w_posture = 1

        # Posture task
        self.postureTask = tsid.TaskJointPosture("task-posture", self.tsid_robot)
        self.postureTask.setKp(K_posture * na_ones)
        self.postureTask.setKd(2.0 * np.sqrt(K_posture) * na_ones)

        self.postureSample = tsid.TrajectorySample(len(q0), len(v0))

        self.formulation.addMotionTask(self.postureTask, w_posture, 1, 0.0)

        # End effector task # Will be defined in execute as it varies on the end effectors
        self.K_ee = K_ee
        self.na_ones = na_ones

        # Joint torque bounds task
        self.actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", self.tsid_robot)
        self.formulation.addActuationTask(self.actuationBoundsTask, 1, 0, 0.0)

        # Joint velocity bounds task
        self.jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", self.tsid_robot, 0.1) # dt will be re-set before executing
        self.formulation.addMotionTask(self.jointBoundsTask, 1, 0, 0.0)

        ## Init the tasks
        self.set_torque_limit(1.0)
        self.set_velocity_limit(1.0)

        ## Create the solver
        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)


    def set_torque_limit(self, scale):
        tau_max = scale * self.tsid_robot.model().effortLimit
        tau_min = - tau_max
        self.actuationBoundsTask.setBounds(tau_min, tau_max)

    def set_velocity_limit(self, scale):
        v_max = scale * self.tsid_robot.model().velocityLimit
        v_min = - v_max
        self.jointBoundsTask.setVelocityBounds(v_min, v_max)

    def execute_path(self, path, commanders, dt):
        # Init end effector tasks
        eeTasks_names = []
        eeTasks = []
        eeSamples = []
        eeIndexes = []
        for targetFrame in path.targetFrames:
            eeIndex = self.robot.pin_robot_wrapper.model.getFrameId(targetFrame)
            if eeIndex >= len(self.robot.pin_robot_wrapper.model.frames):
                rospy.logwarn(F"Frame {targetFrame} not found in the robot model : task related to that frame will be ignored")
                continue

            eeTask_name = "ee-task-" + targetFrame
            eeTask = tsid.TaskSE3Equality(eeTask_name , self.tsid_robot, targetFrame)
            eeTask.setKp(self.K_ee* self.na_ones)
            eeTask.setKd(2.0 * np.sqrt(self.K_ee) * self.na_ones)
            eeTask.useLocalFrame(True) # Represent jacobian in world frame

            eeIndexes.append(eeIndex)
            eeTasks_names.append(eeTask_name)
            eeTasks.append(eeTask)
            eeSamples.append(tsid.TrajectorySample(12, 6)) # Cartesian samples

            self.formulation.addMotionTask(eeTask, self.w_ee, 1, 0.0)

        # Resize solver
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)

        # Update tasks parameters
        self.jointBoundsTask.setTimeStep(dt)

        # Prepare the loop
        rate = rospy.Rate(2. / dt)
        elapsed_time = 0.0
        start_time = None

        # Prepare a filter function
        q_pin_to_hpp, _ , v_pin_to_hpp, _ = self.robot.create_dof_lookup(path.jointList)

        def _rearrange_hpp_to_pin(q, v, v_dot): # Robot._rearrange_ros_to_pin()
                return np.array([q[i] for i in q_pin_to_hpp]), \
                       np.array([v[i] for i in v_pin_to_hpp]), \
                       np.array([v_dot[i] for i in v_pin_to_hpp])

        # Loop
        debug_i = 0
        while elapsed_time < path.corbaPath.length():
            # Measure the current time
            if start_time is None:
                start_time = rospy.Time.now().to_sec()
            else:
                elapsed_time = rospy.Time.now().to_sec() - start_time

            # clip the elapsed time to the path length
            t = min(elapsed_time, path.corbaPath.length())

            # Get references from trajectory
            q     = path.corbaPath.call(t)[0]
            v     = path.corbaPath.derivative(t, 1)
            v_dot = path.corbaPath.derivative(t, 2)

            q, v, v_dot = _rearrange_hpp_to_pin(q,v,v_dot)
            self.robot.display(q)

            # Set posture reference
            self.postureSample.value(q)
            self.postureSample.derivative(v)
            self.postureSample.second_derivative(v_dot)
            self.postureTask.setReference(self.postureSample)

            # Compute forward kinematics
            self.robot.pin_robot_wrapper.forwardKinematics(q, v, v_dot)
            for i in range(len(eeTasks)):
                ee_acc = self.robot.pin_robot_wrapper.frameAcceleration(q, v, v_dot, eeIndexes[i], update_kinematics=False, reference_frame=pin.ReferenceFrame.LOCAL) # No need to recompute the forwad kinematic
                ee_vel = self.robot.pin_robot_wrapper.frameVelocity    (q, v,        eeIndexes[i], update_kinematics=False, reference_frame=pin.ReferenceFrame.LOCAL) # No need to recompute the forwad kinematic
                ee_pos = self.robot.pin_robot_wrapper.framePlacement   (q,           eeIndexes[i], update_kinematics=False) # No need to recompute the forwad kinematic

                ee_acc_vec = ee_acc.vector
                ee_vel_vec = ee_vel.vector
                ee_pos_vec = np.concatenate((ee_pos.translation, ee_pos.rotation.flatten()))

                eeSamples[i].value(ee_pos_vec)
                eeSamples[i].derivative(ee_vel_vec)
                eeSamples[i].second_derivative(ee_acc_vec)

                eeTasks[i].setReference(eeSamples[i])

            q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)
            self.robot.display(q)

            HQPData = self.formulation.computeProblemData(t, np.array(q_meas), np.array(v_meas))
            sol = self.solver.solve(HQPData)
            if(sol.status!=0):
                print(F"Time  {t}  QP problem could not be solved! Error code: {sol.status}")
                print("Posture tasks : ")
                for i in range(len(eeTasks)):
                    print("- eeIndex", eeIndexes[i])
                    print("  eeTasks_names", eeTasks_names[i])
                    print("  ee_pos_vec", eeSamples[i].value())
                    print("  ee_vel_vec", eeSamples[i].derivative())
                    print("  ee_acc_vec", eeSamples[i].second_derivative())
                break


            dv_next = self.formulation.getAccelerations(sol)

            # numerical integration
            v_next = np.array(v_meas + dt*dv_next)
            q_next = pin.integrate(self.robot.pin_robot_wrapper.model,np.array(q_meas),v_next*dt)

            # publish commands
            for commander in commanders:
                commander.execute_step_fwd(v_next)

            # Wait for next step
            rate.sleep()

        # Remove all the end effector tasks
        for taskname in eeTasks_names:
            self.formulation.removeTask(taskname, 0.0)
        # Resize solver
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)