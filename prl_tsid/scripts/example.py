import tsid
import rospy
import numpy as np
import pinocchio as pin
rospy.init_node("TSID", anonymous=True)
from prl_pinocchio.ur5 import robot, commander_left_arm, commander_right_arm

# Init ros node and commanders
commander_left_arm.start_fwd()
commander_right_arm.start_fwd()

# TSID Robot
tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, True, False)

# Base configurations
q0 = np.zeros(tsid_robot.nq)
v0 = np.zeros(tsid_robot.nv)
a0 = np.zeros(tsid_robot.na)

# Forumlation
formulation = tsid.InverseDynamicsFormulationAccForce("tsid", tsid_robot, False)
formulation.computeProblemData(0.0, q0, v0)

qref = np.array([-1.55687287e+00, -1.56574731e+00, -1.56062175e+00, 2.40284589e-02, 1.61412316e+00, -7.95469352e-01, 0., 0., 0., 0., 0., 0., 0., 0., 1.55664212e+00, -1.57508802e+00, 1.56221337e+00, 3.11631454e+00, -1.61955921e+00, 7.73079284e-01, 0., 0., 0., 0., 0., 0.])
vref = np.zeros(tsid_robot.nv)

dt = 0.01
my_K = 100

postureTask = tsid.TaskJointPosture("task-posture", tsid_robot)
postureTask.setKp(my_K* np.ones(tsid_robot.na))
postureTask.setKd(2.0 * np.sqrt(my_K) * np.ones(tsid_robot.na))
formulation.addMotionTask(postureTask, 10, 1, 0.0)

tau_max = tsid_robot.model().effortLimit
tau_min = - tau_max
actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", tsid_robot)
actuationBoundsTask.setBounds(tau_min, tau_max)
formulation.addActuationTask(actuationBoundsTask, 1, 0, 0.0)

v_max = 0.25 * tsid_robot.model().velocityLimit
v_min = - v_max
jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", tsid_robot, dt)
jointBoundsTask.setVelocityBounds(v_min, v_max)
formulation.addMotionTask(jointBoundsTask, 1, 0, 0.0)

trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", qref)
samplePosture = trajPosture.computeNext()

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

robot.create_visualizer()

# Debug
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
pub_err_vel = rospy.Publisher('/tsid/err_vel', Float64MultiArray, queue_size=10)
pub_err_pos = rospy.Publisher('/tsid/err_pos', Float64MultiArray, queue_size=10)

i = 0
r = rospy.Rate(2. / dt)
t_start = rospy.Time.now()
print("Start the loop")
while not rospy.is_shutdown():
    t = rospy.Time.now() - t_start

    # set reference trajectory
    q_ref  = qref# q0 +  amp * np.sin(two_pi_f*t + phi)
    v_ref  = vref # two_pi_f_amp * np.cos(two_pi_f*t + phi)
    # dv_ref[:,i] = -two_pi_f_squared_amp * np.sin(two_pi_f*t + phi)
    samplePosture.value(q_ref)
    samplePosture.derivative(v_ref)
    samplePosture.second_derivative(np.array([0 for _ in v_ref]))
    postureTask.setReference(samplePosture)

    q_meas, v_meas, _ = robot.get_meas_qvtau(raw = True)


    HQPData = formulation.computeProblemData(t.to_sec(), np.array(q_meas), np.array(v_meas))
    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print(F"Time {t} QP problem could not be solved! Error code: {sol.status}")
        break
    
    # tau = formulation.getActuatorForces(sol)
    # if i%25==0:
    #     rospy.logwarn(tau)
    
    # numerical integration
    dv_next = formulation.getAccelerations(sol)
    v_next = np.array(v_meas + 1.0 * dt*dv_next)
    v_mean = np.array(v_meas + 0.5 * dt*dv_next)
    q_next = pin.integrate(robot.pin_robot_wrapper.model, np.array(q_meas), v_mean*dt)

    # v_next = 1 * (q_ref - q_meas)
    # v_mean = (np.array(v_meas) + v_next)/2.
    # q_next = pin.integrate(robot.pin_robot_wrapper.model, np.array(q_meas), v_mean*dt)
    # dv_next = (v_next - v_meas) / dt


    pub_err_vel.publish(Float64MultiArray(data=(v_mean-v_meas).tolist()))
    pub_err_pos.publish(Float64MultiArray(data=(q_ref-q_meas).tolist()))

    # if i%25==0:
    #     rospy.logwarn("{q_meas[i]} - {q_ref[i]} = {q_meas[i] - q_ref[i]}  --> {v_next[i]}")
    #     for i in range(len(q_ref)):
    #         diff = q_meas[i] - q_ref[i]
    #         rospy.logwarn(F"{q_meas[i] : .3f} - {q_ref[i] : .3f} = {diff : .3f}  --> {100 * v_next[i] : .3f}   {'x' if q_ref[i] != 0 and diff*v_next[i] > 0 else ''}")
    #     rospy.logwarn("\n")

    # robot.display(q_next)
    # q_meas, v_meas = q_next, v_next

    # commander_left_arm.execute_step_deprecated(q_meas, v_meas, q_next, v_next, dv_next, dt)
    # commander_right_arm.execute_step_deprecated(q_meas, v_meas, q_next, v_next, dv_next, dt)

    commander_left_arm.execute_step_fwd(v_mean)
    commander_right_arm.execute_step_fwd(v_mean)

    i+=1
    r.sleep()