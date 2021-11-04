from prl_tsid.robots.ur5 import tsid, np, rospy, tsid_robot, formulation, robot, commander_left_arm, commander_right_arm
import pinocchio as pin

qref = np.array([-1.55687287e+00, -1.56574731e+00, -1.56062175e+00, 2.40284589e-02, 1.61412316e+00, -7.95469352e-01, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.55664212e+00, -1.57508802e+00, 1.56221337e+00, 3.11631454e+00, -1.61955921e+00, 7.73079284e-01, 4.43863538e-04, 4.43863538e-04, 4.43863538e-04, 4.43863538e-04, 4.43863538e-04, 4.43863538e-04])
vref = np.zeros(tsid_robot.nv)

my_K = 10

postureTask = tsid.TaskJointPosture("task-posture", tsid_robot)
postureTask.setKp(my_K* np.ones(tsid_robot.na))
postureTask.setKd(2.0 * np.sqrt(my_K) * np.ones(tsid_robot.na))
formulation.addMotionTask(postureTask, 1000, 1, 0.0)


trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", qref)
postureTask.setReference(trajPosture.computeNext())
samplePosture = trajPosture.computeNext()

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)


q, v, _ = robot.get_meas_qvtau()

i = 0
dt = 0.1
r = rospy.Rate(1. / dt)
t = rospy.Time.now()
print("Start the loop")
while True:
    r.sleep()
    t = rospy.Time.now()

    # set reference trajectory
    q_ref  = qref# q0 +  amp * np.sin(two_pi_f*t + phi)
    v_ref  = vref # two_pi_f_amp * np.cos(two_pi_f*t + phi)
    # dv_ref[:,i] = -two_pi_f_squared_amp * np.sin(two_pi_f*t + phi)
    samplePosture.value(q_ref)
    samplePosture.derivative(v_ref)
    samplePosture.second_derivative(np.array([0 for _ in v]))
    postureTask.setReference(samplePosture)

    r.sleep()
    q_meas, v_meas, _ = robot.get_meas_qvtau(raw = True)

    if(i%10 == 0):
        rospy.logwarn(q)

    HQPData = formulation.computeProblemData(t.to_sec(), np.array(q_meas), np.array(v_meas))
    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print("Time %.3f QP problem could not be solved! Error code:"%t, sol.status)
        break
    
    # tau = formulation.getActuatorForces(sol)
    dv_next = formulation.getAccelerations(sol)

    # numerical integration
    v_next = np.array(v_meas + dt*dv_next)
    q_next = pin.integrate(robot.pin_model,np.array(q),v_next*dt)

    commander_left_arm.execute(q_next, v_next, dv_next, dt)
    commander_right_arm.execute(q_next, v_next, dv_next, dt)

    i+=1