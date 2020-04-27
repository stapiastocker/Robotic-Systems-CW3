# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
from cell import CellLabel
from search_grid import SearchGrid
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

        for waypoint in self.currentPlannedPath.waypoints:
            coordinates = waypoint.coords
            if (self.occupancyGrid.getCell(coordinates[0], coordinates[1])==1):
                self.controller.stopDrivingToCurrentGoal()
                self.obstacle = coordinates
                print("Collision detected at:", coordinates)
                print("New Path Generated.")

    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        p_b = 0.8
        L_w = 2.0
        lambdaB_max = 101.0 * p_b * L_w/102.0
        smallestLength = float('inf')
        chosenAisle = Aisle.B

        for aisle in Aisle:
            path = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
            length = path.numberOfWaypoints
            if aisle == Aisle.B:
                length += p_b*L_w/lambdaB_max
            print(f"Aisle: {aisle}, cost {length}")
            if length < smallestLength:
                chosenAisle = aisle
                smallestLength = length

        return chosenAisle


    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.A

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords, waitingTime = 2):
        # Cost of waiting
        T = waitingTime # needs to be changed to equation with lambda_b
        Lw = 2.0
        costOfWaiting = T * Lw
        rospy.loginfo("Waiting Cost: {}".format(costOfWaiting))

        # Cost of original route
        waypoints = self.currentPlannedPath.waypoints
        waypoints = [waypoint.coords for waypoint in self.currentPlannedPath.waypoints]
        waypoints.reverse()
        pathCost = 0

        print("Start Cell Coords: ", startCellCoords)

        for waypoint in waypoints:
            if waypoint != startCellCoords: #, waypoint[0] != (startCellCoords[0]) or waypoint[1] != startCellCoords[1]
                print(waypoint)
                pathCost += 1
            else:
                break
        rospy.loginfo("Old Path Travel Cost: {}".format(pathCost))

        # Cost of new route
        newAisle = self.chooseAisle(startCellCoords, goalCellCoords)
        newPath = self.planPathToGoalViaAisle(startCellCoords,goalCellCoords, newAisle)
        newPathCost = newPath.numberOfWaypoints
        rospy.loginfo("New Path Travel Cost: {}".format(newPathCost))

        rospy.loginfo("Path Cost Difference: {}".format(newPathCost - pathCost))

        # Maximum value of lambda_b
        lambda_b_max = Lw / (newPathCost - pathCost)
        rospy.loginfo("Lamba_b Max: {}".format(lambda_b_max))

        # Which relates to a mean waiting time of
        meanWaitingTime = 1.0/lambda_b_max
        rospy.loginfo("Giving a mean waiting time of: {}".format(meanWaitingTime))

        if costOfWaiting < newPathCost - pathCost:
            rospy.loginfo("Action: Wait")
            return True
        else:
            rospy.loginfo("Action: Replan")
            return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):

        while True:
            if (self.occupancyGrid.getCell(self.obstacle[0], self.obstacle[1])==1):
                continue
            else:
                rospy.loginfo("Obstacle cleared.")
                break
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle
        elif self.aisleToDriveDown != aisle:
            self.aisleToDriveDown = aisle

        # Implement your method here to construct a path which will drive the robot
        # from the start to the goal via the aisle.

        # Path through chosen aisle
        if self.aisleToDriveDown.value == 0:
            print("Drive down A")
            aisleCoords = (28, 12)
        elif self.aisleToDriveDown.value == 1:
            print("Drive down B")
            aisleCoords = (42, 12)
        elif self.aisleToDriveDown.value == 2:
            print("Drive down C")
            aisleCoords = (57, 12)
        elif self.aisleToDriveDown.value == 3:
            print("Drive down D")
            aisleCoords = (72, 12)
        elif self.aisleToDriveDown.value == 4:
            print("Drive down E")
            aisleCoords = (87, 12)

        pathToAisle = self.planner.search(startCellCoords, aisleCoords)
        # If we can't reach the goal, give up and return
        if pathToAisle is False:
            rospy.logwarn("Could not find a path to the aisle at (%d, %d)", \
                            aisleCoords[0], aisleCoords[1])
            return None

        currentPlannedPath = self.planner.extractPathToGoal()

        pathToGoalFound = self.planner.search(aisleCoords, goalCellCoords) 
        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None

        # Extract the path
        currentPlannedPath.addToEnd(self.planner.extractPathToGoal())

        return currentPlannedPath

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        print("Goal", goal)
        print("Goal x", goal.x)
        print("Goal y", goal.y)

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            print("Aisle to Drive down:", aisleToDriveDown)
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False

            self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(self.currentPlannedPath, 'yellow')
            self.planner.searchGridDrawer.waitForKeyPress()

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords, waitingTime=3)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
