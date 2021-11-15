import tsid
import rospy
import numpy as np
import pinocchio as pin

class PathFollower:
    def __init__(self, robot):
        # Init the problem
        self.robot = robot
        self.tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, True, False)
        self.formulation = tsid.InverseDynamicsFormulationAccForce("tsid", self.tsid_robot, False)

        # Some default values
        q0 = np.zeros(self.tsid_robot.nq)
        v0 = np.zeros(self.tsid_robot.nv)
        a0 = np.zeros(self.tsid_robot.na)

        self.formulation.computeProblemData(0.0, q0, v0)

        # Create a vizualizer
        self.robot.create_visualizer()
        self.robot.display(q0)

        # Create the tasks
        my_K = 0.1

        self.postureSample = tsid.TrajectorySample(len(q0), len(v0))
        self.postureTask = tsid.TaskJointPosture("task-posture", self.tsid_robot)
        self.postureTask.setKp(my_K* np.ones(self.tsid_robot.na))
        self.postureTask.setKd(2.0 * np.sqrt(my_K) * np.ones(self.tsid_robot.na))
        self.formulation.addMotionTask(self.postureTask, 1, 1, 0.0)

        self.actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", self.tsid_robot)
        self.formulation.addActuationTask(self.actuationBoundsTask, 1, 0, 0.0)
        
        self.jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", self.tsid_robot, 0.1)
        self.formulation.addMotionTask(self.jointBoundsTask, 1, 0, 0.0)

        # Init the tasks
        self.set_torque_limit(1.0)
        self.set_velocity_limit(1.0)

        # Create the solver
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
        # Update tasks parameters
        self.jointBoundsTask.setTimeStep(dt)

        # Prepare the loop
        rate = rospy.Rate(2. / dt)
        elapsed_time = 0.0
        start_time = None

        # Loop
        while elapsed_time < path.corbaPath.length():
            # Measure the current time
            if start_time is None:
                start_time = rospy.Time.now().to_sec()
            else:
                elapsed_time = rospy.Time.now().to_sec() - start_time

            # clip the elapsed time to the path length
            t = min(elapsed_time, path.corbaPath.length())
            
            # set reference trajectory
            q     = np.array(path.corbaPath.call(t)[0])
            v     = np.array(path.corbaPath.derivative(t, 1))
            v_dot = np.array(path.corbaPath.derivative(t, 2))
            self.postureSample.value(q)
            self.postureSample.derivative(v)
            self.postureSample.second_derivative(v_dot)
            self.postureTask.setReference(self.postureSample)

            q_meas, v_meas, _ = self.robot.get_meas_qvtau(raw = True)

            self.robot.display(np.array(q_meas))

            HQPData = self.formulation.computeProblemData(t, np.array(q_meas), np.array(v_meas))
            sol = self.solver.solve(HQPData)
            if(sol.status!=0):
                print(F"Time {t} QP problem could not be solved! Error code: {sol.status}")
                break
            
            # tau = formulation.getActuatorForces(sol)
            # if i%25==0:
            #     rospy.logwarn(tau)
            
            dv_next = self.formulation.getAccelerations(sol)

            # numerical integration
            v_next = np.array(v_meas + dt*dv_next)
            q_next = pin.integrate(self.robot.pin_robot_wrapper.model,np.array(q_meas),v_next*dt)

            # publish commands
            for commander in commanders:
                commander.execute_step(q_next, v_next, dv_next, dt)
            
            # Wait for next step
            rate.sleep()