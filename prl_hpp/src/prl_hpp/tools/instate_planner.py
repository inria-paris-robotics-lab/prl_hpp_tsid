from prl_hpp.tools.utils import wd
from CORBA import Any, TC_long, TC_float

class InStatePlanner:
    # Default path planner
    parameters = {'kPRM*/numberOfNodes': Any(TC_long,2000)}

    def __init__(self, ps):
        self.ps = ps
        self.plannerType = "BiRRT*"
        self.optimizerTypes = []
        self.maxIterPathPlanning = None
        self.timeOutPathPlanning = None
        self.stopWhenPathIsSolved = None

        self.manipulationProblem = wd(self.ps.hppcorba.problem.getProblem())
        self.crobot = self.manipulationProblem.robot()
        # create a new problem with the robot
        self.cproblem = wd(self.ps.hppcorba.problem.createProblem(self.crobot))
        # Set parameters
        for k, v in self.parameters.items():
            self.cproblem.setParameter(k, v)
        # Set config validation
        self.cproblem.clearConfigValidations()
        for cfgval in [ "CollisionValidation", "JointBoundValidation"]:
            self.cproblem.addConfigValidation(wd(self.ps.hppcorba.problem.createConfigValidation(cfgval, self.crobot)))
        # get reference to constraint graph
        self.cgraph = self.manipulationProblem.getConstraintGraph()
        # Add obstacles to new problem
        for obs in self.ps.getObstacleNames(True,False):
            self.cproblem.addObstacle(wd(self.ps.hppcorba.problem.getObstacle(obs)))

    def setEdge(self, graph, edge):
        # Get constraint of edge
        edgeLoopFree = wd(self.cgraph.get(graph.edges[edge]))
        self.cconstraints = wd(edgeLoopFree.pathConstraint())
        self.cproblem.setPathValidation(edgeLoopFree.getPathValidation())
        self.cproblem.setConstraints(self.cconstraints)
        self.cproblem.setSteeringMethod(wd(edgeLoopFree.getSteeringMethod()))
        self.cproblem.filterCollisionPairs()

    def setReedsAndSheppSteeringMethod(self):
        sm = wd(self.ps.hppcorba.problem.createSteeringMethod("ReedsShepp", self.cproblem))
        self.cproblem.setSteeringMethod(sm)
        dist = self.ps.hppcorba.problem.createDistance("ReedsShepp", self.cproblem)
        self.cproblem.setDistance(dist)

    def buildRoadmap(self, qinit):
        wd(self.cconstraints.getConfigProjector()).setRightHandSideFromConfig(qinit)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        self.roadmap = wd(self.ps.client.manipulation.problem.createRoadmap(
                wd(self.cproblem.getDistance()),
                self.crobot))
        self.roadmap.constraintGraph(self.cgraph)
        self.planner = wd(self.ps.hppcorba.problem.createPathPlanner(
            self.plannerType,
            self.cproblem,
            self.roadmap))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        if self.stopWhenPathIsSolved:
            self.planner.stopWhenProblemIsSolved(self.stopWhenPathIsSolved)
        path = wd(self.planner.solve())

    def createEmptyRoadmap(self):
        self.roadmap = wd(self.ps.hppcorba.problem.createRoadmap(
                wd(self.cproblem.getDistance()),
                self.crobot))

    def computePath(self, qinit, qgoals, resetRoadmap = False):
        wd(self.cconstraints.getConfigProjector()).setRightHandSideFromConfig(qinit)
        for qgoal in qgoals:
            res, qgoal2 = self.cconstraints.apply(qgoal)
            self.cproblem.setInitConfig(qinit)
            self.cproblem.resetGoalConfigs()
            self.cproblem.addGoalConfig(qgoal2)

        if resetRoadmap or not hasattr(self, 'roadmap'):
            self.createEmptyRoadmap()
        self.planner = wd(self.ps.hppcorba.problem.createPathPlanner(self.plannerType, self.cproblem, self.roadmap))

        self.planner.stopWhenProblemIsSolved(True)
        self.planner.maxIterations(-1)
        self.planner.timeOut(float("inf"))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        if self.stopWhenPathIsSolved:
            self.planner.stopWhenProblemIsSolved(self.stopWhenPathIsSolved)

        path = wd(self.planner.solve())
        for optType in self.optimizerTypes:
            optimizer = wd(self.ps.hppcorba.problem.createPathOptimizer(optType, self.manipulationProblem))
            try:
                optpath = optimizer.optimize(path)
                # optimize can return path if it couldn't find a better one.
                # In this case, we have too different client refering to the same servant.
                # thus the following code deletes the old client, which triggers deletion of
                # the servant and the new path points to nothing...
                # path = wd(optimizer.optimize(path))
                from hpp.corbaserver.tools import equals
                if not equals(path, optpath):
                    path = wd(optpath) # Do something to avoid corba error
            except HppError as e:
                print("could not optimize", e)
        return path
