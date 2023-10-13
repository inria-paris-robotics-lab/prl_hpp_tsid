from hpp.corbaserver.manipulation import ProblemSolver, Rule, Constraints, ConstraintGraph, ConstraintGraphFactory, Client
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import PathPlayer
from hpp.corbaserver import loadServerPlugin
from hpp_idl.hpp import Error as HppError

from prl_pinocchio.tools.utils import compare_configurations, compare_poses, euler_to_quaternion
from prl_hpp.tools.utils import wd
from prl_hpp.tools.instate_planner import InStatePlanner

from prl_hpp.tools.hpp_robots import HppRobot, TargetRobotStrings, SupportRobotStrings

try:
    loadServerPlugin ("corbaserver", "manipulation-corba.so")
except:
    import os
    loadServerPlugin ("corbaserver", os.environ.get('CONDA_PREFIX')+"/lib/hppPlugins/manipulation-corba.so")

Client ().problem.resetProblem ()

class Path:
    def __init__(self, id, corbaPath, jointList, targetFrames = []):
        self.id = id
        self.corbaPath = corbaPath
        self.jointList = jointList
        self.targetFrames = targetFrames

class Planner:
    """
    Wrap around HPP to simplify common plannig.
    """

    def __init__(self, robot):
        """
        Parameters
        ----------
            robot (Robot): Associated Robot class. (see robot.py)
        """
        self.robot = robot
        self.hpp_robot = HppRobot(robot.pin_robot_wrapper.model.name, "robot", robot.get_urdf_explicit(), robot.get_srdf_explicit())

        # Problem
        self.ps = ProblemSolver(self.hpp_robot)
        self.graph = wd(self.hpp_robot.client.manipulation.graph)

        # Viewer
        self.vf = ViewerFactory(self.ps)
        self.v = self.vf.createViewer()
        self.pp = PathPlayer(self.v)

        # Add the objects
        self._create_target("target", 0.1) #TODO: be able to change this arbitrary value
        self._create_support("support_pick", 0.1)
        self._create_support("support_place", 0.1)

        # Planning parameters
        self.ps.setErrorThreshold (1e-3)
        self.ps.setMaxIterProjection (40)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)
        self.set_planning_timeout(10.0)

        # Time parametrization
        self.velocity_scale = 1.0
        self.acceleration_scale = 1.0

        # User defined locked joint constraint (re-used for every graph)
        self.lockJointConstraints = []

    def set_planning_timeout(self, timeout, stopWhenProblemIsSolved = True):
        """
        Set the maximum duration the planner has to find a solution.

        Parameters
        ----------
            timeout (float): Timeout duration (in s.)

        Optionnals parameters:
        ----------------------
            stopWhenProblemIsSolved (bool): If set to True, the planner stop as soon as a path is found. If set to False, continues up to the timeout anyway.
        """
        self.ps.setTimeOutPathPlanning(timeout)
        self.stopWhenProblemIsSolved = stopWhenProblemIsSolved # Keep this value as it cannot be read from ps
        # self.ps.stopWhenProblemIsSolved(stopWhenProblemIsSolved)

    def lock_joints(self, jointNames, jointValues = None, constraintNames = None, lockName = "locked_joints"):
        """
        Lock joints that shouldn't be controlled.

        Create lock joints constraints and add it to the problem solver.

        Parameters
        ----------
            jointNames (str[]): Names of the joints to lock (with robotName prefix).

        Optionnals parameters:
        ----------------------
            jointValues (int[]): Values to lock the joints in. If unspecified lock the joint at the current values.
            constraintNames (str[]): Constraints name.
            lockName (str): Name of the lockJoint constraint.

        Returns
        -------
            constraintNames (str[]): name of created constraints.
        """
        # Generate constraints name
        if constraintNames == None:
            constraintNames = ["locked_" + jointName for jointName in jointNames]

        # get locked joint values from current configuration
        if jointValues == None:
            all_names = self.robot.get_joint_names()
            q_current = self.robot.get_meas_q()
            jointValues = []
            for j_name in jointNames:
                j_index = list(self.robot.pin_robot_wrapper.model.names).index(j_name)
                j_idx_q = self.robot.pin_robot_wrapper.model.joints[j_index].idx_q
                j_nq = self.robot.pin_robot_wrapper.model.joints[j_index].nq
                jointValues.extend(q_current[j_idx_q:j_idx_q+j_nq])

        # Create the constraints
        for i in range(len(jointNames)):
            self.ps.createLockedJoint(constraintNames[i], "robot/" + jointNames[i], [jointValues[i]])

        self.ps.addLockedJointConstraints(lockName, constraintNames)

        self.lockJointConstraints.extend(constraintNames)
        return constraintNames

    def make_gripper_approach(self, gripperName, pose, approach_distance = 0.1, q_start = None, validate = True, do_not_plan = False):
        """
        Find a path that leads to the gripper at a certain pose with a certain approach direction.

        Parameters
        ----------
            gripperName (str): Names of the gripper (without robotName prefix), as defined in srdf.
            pose ([float[3], float[3 or 4]]): Desired pose (position, orientation) of the tool in world frame. (orientation can be euler angles or quaternions).

        Optionnals parameters:
        ----------------------
            approach_distance (float): Distance from which the gripper should make a quasi-straight to reach the goal pose.
            q_start (foat[]): The initial configuration of the robot. (If unspecified, the initial configuration is set to the current one.)
            validate (bool): If true, will check if the constraint graph generated is valid.
            do_not_plan (bool): If true, will not plan, but will generate the constraint graph with goal configurations etc. (useful to check if a goal pose is 'do-able'. Exception will be thrown if not)
        Returns
        -------
            path (Path): the path found

        Raises
        ------
            AssertionError: If the start configuration is not valid.
            AssertionError: If no valid goal configuration can be found.
            AssertionError: If validate is True and the graph is not found valid.
        """
        if q_start == None:
            q_start = self.robot.get_meas_q()

        pose[1] = self._convert_orientation(pose[1])

        self._reset_problem()
        gripperFullname = "robot/" + gripperName
        gripperLink = self.robot.get_gripper_link(gripperName)

        # The configuration space is now bigger because of the configuration of the target
        q_start = self._merge_q(q_start, q_target = (pose[0]+pose[1]) )

        # Create the ConstraintGraph
        cg = self._create_simple_cg([gripperFullname], ['target'], [['target/handle']], validate, replace_target_constraints=True)

        # Project the initial configuration in the initial node
        res_init, q_init, _ = cg.applyNodeConstraints("free", q_start)
        assert res_init, "Initial configuration is not valid"

        # Display start configuration
        self.v(q_init)

        # Generate pair goal configuration (pre-graps, grasp)
        q_goals = []
        for _ in range(100):
            q = self.hpp_robot.shootRandomConfig()
            res_pre, q_pre, error_pre = cg.generateTargetConfig(gripperFullname + ' > target/handle | f_01', q_init, q)
            res_grasp, q_grasp, error_grasp = cg.generateTargetConfig(gripperFullname + ' > target/handle | f_12', q_pre, q_pre)
            if(res_pre and res_grasp):
                res_path, pathId, error_path = self.ps.directPath(q_pre, q_grasp, True) # Check that a collision free direct path is feasible
                collide, _ = self.robot.compute_collisions(self._split_q(q_grasp, 'robot'), stop_at_first_collision=True) # if the path is of length 0, the collision wouldn't be checked
                if not collide and res_path:
                    q_goals.append([q_pre, q_grasp])
                self.ps.erasePath(pathId) # Erase the path as it's not needed anymore

        assert len(q_goals) > 0, "No goal configuration found"

        # Check if should plan or not
        if do_not_plan:
            return

        # Prepare solving in-state
        instatePlanner = InStatePlanner (self.ps)
        instatePlanner.setEdge(cg, "Loop | f")
        instatePlanner.optimizerTypes = [ "RandomShortcut" ]
        instatePlanner.timeOutPathPlanning = self.ps.getTimeOutPathPlanning()
        instatePlanner.stopWhenProblemIsSolved = self.stopWhenProblemIsSolved

        # Solve the problem
        path = instatePlanner.computePath(q_init, [q_pre for q_pre, q_grasp in q_goals], resetRoadmap=True)

        # Add path to the problem
        pathId = self.ps.hppcorba.problem.addPath(path)

        # Find which pre-grasp configuration is at the end of the path
        q_end = path.end()
        q_end_grasp = None
        for q_pre, q_grasp in q_goals:
            if(compare_configurations(self.robot.pin_robot_wrapper.model, self._split_q(q_pre, 'robot'), self._split_q(q_end, 'robot'))):
                q_end_grasp = q_grasp
                break
        assert q_end_grasp != None, "Error while concatenating the last part of the path."

        # Add the pre-grasp to grap path
        wps = self.ps.getWaypoints(pathId)[0]
        pathId = self._create_path(wps + [q_end_grasp]) # Do this in such way so null length segment are removed from the path (to prevent latter hpp bug during time parameterization)

        # Time parametrization of the path
        paramPathId = self._timeParametrizePath(pathId)
        paramPath = wd(self.ps.hppcorba.problem.getPath(paramPathId))

        # Delete the first path as it won't be used anymore
        path.deleteThis()

        # return path
        return Path(paramPathId, paramPath, self.robot.get_joint_names(), [gripperLink])

    def set_velocity_limit(self, scale):
        """
        Set the velocity limit for time parametrization of paths (relatively to the max joint velocity of the robot).

        Parameters
        ----------
            scale (float): Desired ratio of limit_velocity / max_velocity.
        """
        self.velocity_scale = scale

    def set_acceleration_limit(self, scale):
        """
        Set the acceleration limit for time parametrization of paths (relatively to the max joint acceleration of the robot).

        Parameters
        ----------
            scale (float): Desired ratio of limit_acceleration / max_acceleration.
        """
        self.acceleration_scale = scale

    def make_pick_and_place(self, gripperName, pose_pick, pose_place, approach_distance = 0.1, q_start = None, q_end = None, validate = True):
        """
        Find 3 paths to pick an object, place the object and go to end position.

        Parameters
        ----------
            gripperName (str): Names of the gripper (without robotName prefix), as defined in srdf.
            pose_pick ([float[3], float[3 or 4]]): Picking pose (position, orientation) of the tool in world frame. (orientation can be euler angles or quaternions).
            pose_place ([float[3], float[3 or 4]]): Placing pose (position, orientation) of the tool in world frame. (orientation can be euler angles or quaternions).

        Optionnals parameters:
        ----------------------
            approach_distance (float): Distance from which the gripper should make a quasi-straight line pick (and place) the object.
            q_start (foat[]): The initial configuration of the robot. (If unspecified, the initial configuration is set to the current one.)
            q_end (foat[]): The desired final configuration of the robot. (If unspecified, the final configuration will be the same as the initial one.)
            validate (bool): If true, will check if the constraint graph generated is valid.

        Returns
        -------
            paths (Path[3 (or 0)]): the path picking, placing and homing paths. If he list is empty, no path was found in the timeout duration.

        Raises
        ------
            AssertionError: If the start configuration is not valid.
            AssertionError: If the end configuration is not valid.
            AssertionError: If validate is True and the graph is not found valid.
        """
        if q_start == None:
            q_start = self.robot.get_meas_q()
        if q_end == None:
            q_end = q_start[:]

        # Convert euler rotations into quaternions (if needed)
        pose_pick[1]  = self._convert_orientation(pose_pick[1])
        pose_place[1] = self._convert_orientation(pose_place[1])

        # Merge postion and orientation in one list
        pose_pick = pose_pick[0] + pose_pick[1]
        pose_place = pose_place[0] + pose_place[1]

        self._reset_problem()
        gripperFullname = "robot/" + gripperName
        gripperLink = self.robot.get_gripper_link(gripperName)

        # The configuration space is bigger because of the configuration of pick and place objects
        q_start = self._merge_q(q_start, q_target = pose_pick)
        q_end = self._merge_q(q_end, q_target = pose_place)

        # Rules
        rules = [
            Rule(["support_pick/gripper", "support_place/gripper"], ["target/handle_bottom", "target/handle"], False),
            Rule(["support_pick/gripper", "support_place/gripper"], ["target/handle", "target/handle_bottom"], False),
            Rule([gripperFullname], ["target/handle_bottom"], False),

            Rule([gripperFullname], ["target/handle"], True),
            Rule(["support_pick/gripper"],  ["target/handle_bottom"], True),
            Rule(["support_place/gripper"], ["target/handle_bottom"], True),
        ]

        # Place supports at pick and place locations
        self.hpp_robot.setJointPosition('support_pick/root_joint', pose_pick)
        self.hpp_robot.setJointPosition('support_place/root_joint', pose_place)

        # Create the ConstraintGraph
        cg = self._create_simple_cg([gripperFullname, "support_pick/gripper", "support_place/gripper"], ['target', "support_pick", "support_place"], [['target/handle', 'target/handle_bottom'], [], []], validate, rules = rules)

        # Project the initial configuration in the initial node
        res_init, q_init, _ = cg.applyNodeConstraints("support_pick/gripper grasps target/handle_bottom", q_start)
        assert res_init, "Initial configuration is not valid"
        self.ps.setInitialConfig(q_init)

        self.v(q_init)

        # Generate pair goal configuration (pre-graps, grasp)
        q_goals = []
        for _ in range(100):
            q = self.hpp_robot.shootRandomConfig()
            res_place, q_place, error_place = cg.applyNodeConstraints(gripperFullname + ' grasps target/handle : support_place/gripper grasps target/handle_bottom', q)
            res_clear, q_clear, error_clear = cg.generateTargetConfig(gripperFullname +' < target/handle | 0-0:2-1_21', q_place, q_place)
            if(res_place and res_clear):
                res_path, pathId, error_path = self.ps.directPath(q_place, q_clear, True) # Check that a collision free direct path is feasible
                collide, _ = self.robot.compute_collisions(self._split_q(q_clear, 'robot'), stop_at_first_collision=True) # if the path is of length 0, the collision wouldn't be checked
                if res_path:
                    q_goals.append([q_place, q_clear])
                    self.ps.addGoalConfig(q_place)
                self.ps.erasePath(pathId) # Erase the path as it's not needed anymore
        assert len(q_goals) > 0, "No goal configuration found"
        # Solve the problem
        success = self._safe_solve()
        if not success:
            return [] # No path found
        pathId = self.ps.numberPaths()-1

        # Optimize path
        self.ps.clearPathOptimizers()
        self.ps.addPathOptimizer("EnforceTransitionSemantic")
        self.ps.addPathOptimizer("RandomShortcut")
        self.ps.optimizePath(pathId)
        pathId = self.ps.numberPaths()-1

        # Split the path
        wps = self.ps.getWaypoints(pathId)[0]
        pick_index = None
        for i, q in enumerate(wps):
            target_pose = self._split_q(q, 'target')
            if(not compare_poses(target_pose, pose_pick)):
                pick_index = i
                break

        # Create sub path
        pick_pathId = self._create_path(wps[:pick_index])
        place_pathId = self._create_path(wps[pick_index-1:])

        # Find which pre-grasp configuration is at the end of the path
        q_end = wps[-1]
        q_end_clear = None
        for q_place, q_clear in q_goals:
            if(compare_configurations(self.robot.pin_robot_wrapper.model, self._split_q(q_place, 'robot'), self._split_q(q_end, 'robot'))):
                q_end_clear = q_clear
                break
        assert q_end_clear != None, "Error while concatenating the last part of the path."

        # Create the homing path
        home_pathId = self._create_path([q_end, q_end_clear])

        # Time parametrization of the paths
        param_pick_pathId = self._timeParametrizePath(pick_pathId)
        param_place_pathId = self._timeParametrizePath(place_pathId)
        param_home_pathId = self._timeParametrizePath(home_pathId)

        # Extract paths
        param_pick_path = wd(self.ps.hppcorba.problem.getPath(param_pick_pathId))
        param_place_path = wd(self.ps.hppcorba.problem.getPath(param_place_pathId))
        param_home_path = wd(self.ps.hppcorba.problem.getPath(param_home_pathId))

        # return paths
        return  Path(param_pick_pathId, param_pick_path, self.robot.get_joint_names(), [gripperLink]), \
                Path(param_place_pathId, param_place_path, self.robot.get_joint_names(), [gripperLink]), \
                Path(param_home_pathId, param_home_path, self.robot.get_joint_names(), [gripperLink])

    def display(self, q_robot, pose_target=None, pose_pick=None, pose_place=None):
        # If no target pose is specified, retreive the one currently displayed
        if(pose_target is None):
            if(hasattr(self.v, 'robotConfig')):
                pose_target = self._split_q(self.v.robotConfig, 'target')
            else: # If no configuration has ever been set before
                pose_target = [0,0,0, 0,0,0,1]

        # Position pick and place markers if necessary
        if(pose_pick is not None):
            self.hpp_robot.setJointPosition('support_pick/root_joint', pose_pick)
        if(pose_place is not None):
            self.hpp_robot.setJointPosition('support_place/root_joint', pose_place)

        # Display robot and target
        self.v(self._merge_q(q_robot, pose_target))

    def _merge_q(self, q_robot, q_target = [0,0,0, 0,0,0,1]):
        return (q_robot + q_target)

    def _split_q(self, q, part=''):
        split = {'robot': q[:-7], 'target': q[-7:]}
        return split if part=='' else split[part]

    def _create_target(self, targetName, clearance, bounds = [-2, 2]*3 + [-1, 1]*4, double_handle = False):
        target = TargetRobotStrings(clearance, double_handle = double_handle)

        # Create the target object target
        self.vf.loadRobotModelFromString(targetName, 'freeflyer', target.urdf, target.srdf)

        # Bound the pickTarget pose around the desired pose
        self.hpp_robot.setJointBounds (targetName + '/root_joint', bounds)

        # Re-create the viewer with the target
        self.v = self.vf.createViewer()
        self.pp = PathPlayer(self.v)

    def _create_support(self, supportName, clearance):
        support = SupportRobotStrings(clearance)

        # Create the support object
        self.vf.loadRobotModelFromString(supportName, 'anchor', support.urdf, support.srdf)

        # Re-create the viewer with the target
        self.v = self.vf.createViewer()
        self.pp = PathPlayer(self.v)

    def _reset_problem(self):
        try:
            self.graph.deleteGraph('graph')
        except HppError:
            pass
        self.ps.clearRoadmap()
        self.ps.resetGoalConfigs()

    def _timeParametrizePath(self, pathId):
        self.ps.clearPathOptimizers()
        self.ps.addPathOptimizer("SimpleTimeParameterization")

        self.ps.setParameter("SimpleTimeParameterization/safety", self.velocity_scale) # velocity limit factor
        self.ps.setParameter("SimpleTimeParameterization/order", 2)
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", self.robot.MAX_JOINT_ACC * self.acceleration_scale)
        # self.ps.setParameter("ManipulationPlanner/extendStep", 0.7)
        # self.ps.setParameter("SteeringMethod/Carlike/turningRadius", 0.05)

        self.ps.optimizePath(pathId)
        return self.ps.numberPaths() -1

    def _convert_orientation(self, orientation):
        """
        Return the orientation as quaternions
        """
        if(len(orientation) == 3):
            return euler_to_quaternion(orientation)
        return orientation

    def _create_path(self, waypoints):
        # Ensure that there are no null length segments in the path (to prevent latter hpp bug during time parametrization)
        model = self.robot.pin_robot_wrapper.model
        nq = model.nq
        filtered_waypoints = []
        for i in range(len(waypoints)-1):
            if( not compare_configurations(model, self._split_q(waypoints[i], 'robot'), self._split_q(waypoints[i+1], 'robot')) ):
                filtered_waypoints.append(waypoints[i])
        filtered_waypoints.append(waypoints[-1])

        # Create path with remaining waypoints
        _, pathId, _ = self.ps.directPath(filtered_waypoints[0], filtered_waypoints[1], False)
        for q in filtered_waypoints[2:]:
            self.ps.appendDirectPath(pathId, q, False)
        return pathId

    def _create_simple_cg(self, grippers, objects, objects_handles, validate, replace_target_constraints = False, rules = None):
        # Rules
        if(rules is None):
            rules = [ Rule([".*"], [".*"], True), ]

        # Create graph using ConstraintGraphFactory
        cg = ConstraintGraph (self.hpp_robot, 'graph')
        factory = ConstraintGraphFactory (cg)
        factory.setGrippers (grippers)
        factory.setObjects (objects, objects_handles, [[None]] * len(objects))
        factory.setRules (rules)
        factory.generate ()
        cg.addConstraints (graph=True, constraints= Constraints(numConstraints = self.lockJointConstraints))

        cproblem = wd(self.ps.client.basic.problem.getProblem())

        # Replace box locked joints constraint of the targets with locked transform constraint to avoid later problems
        if(replace_target_constraints):
            cgraph = cproblem.getConstraintGraph()
            for obj_i, obj in enumerate(objects):
                for handle in objects_handles[obj_i]:
                    constrain_name = 'implicit Transform '+obj+'/root_joint'
                    self.ps.client.basic.problem.createTransformationR3xSO3Constraint(constrain_name, '',''+obj+'/root_joint', [0,0,0, 0,0,0,1], [0,0,0, 0,0,0,1], [True, True, True, True, True, True,])
                    self.ps.setConstantRightHandSide(constrain_name, False)
                    for gripper in grippers: # TODO: Test this nested loop inner code in the case of multiple grippers and targets (is there any edges that haven't been updated ?)
                        for edge_name_suffix in ['> '+handle+' | f_01', '> '+handle+' | f_12', '< '+handle+' | 0-0_10', '< '+handle+' | 0-0_21']:
                            edge_name = gripper + ' ' + edge_name_suffix
                            # edge_nb = cg.edges[edge_name]
                            # edge = cgraph.get(edge_nb)
                            # edge.resetNumericalConstraints() # TODO: To be removed ? (What if multiple grippers and objects : are we deleting too much ?)
                            cg.addConstraints(edge=edge_name, constraints = Constraints(numConstraints=[constrain_name]))

        # Initialize
        factory.generate ()
        cg.initialize()

        # Validate constraint graph
        if(validate):
            cgraph = cproblem.getConstraintGraph()
            cgraph.initialize()
            graphValidation = wd(self.ps.client.manipulation.problem.createGraphValidation())
            assert graphValidation.validate(cgraph), graphValidation.str()

        return cg

    def _safe_solve(self):
        """
        Solve the problem and catch timeout exception
        """
        try:
            self.ps.solve()
        except HppError as e:
            # print(e.msg)
            return False
        return True