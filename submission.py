import re, util

############################################################
# Problem 1a: UCS test case

# Return an instance of util.SearchProblem.
# You might find it convenient to use
# util.createSearchProblemFromString.
def createUCSTestCase(n):
    # BEGIN_YOUR_CODE (around 5 lines of code expected)
    #raise Exception("Not implemented yet")
    edgeString = ""
    for i in range(0,n):
        newNode = "A"+str(i)
        edgeString+="S "+ newNode+" "+ str(util.random.randint(1,10))+"\n"+newNode +" G "+str(util.random.randint(1,10))+"\n"
    return util.createSearchProblemFromString("S", "G", edgeString)
    # END_YOUR_CODE

############################################################
# Problem 1b: A-star search

# Takes the SearchProblem |problem| you're trying to solve and a |heuristic|
# (which is a function that maps a state to an estimate of the cost to the
# goal).  Returns another search problem |newProblem| such that running uniform
# cost search on |newProblem| is equivalent to running A* on |problem| with
# |heuristic|.
def astarReduction(problem, heuristic):
    class NewSearchProblem(util.SearchProblem):
        # Please refer to util.SearchProblem to see the functions you need to
        # overried.
        # BEGIN_YOUR_CODE (around 9 lines of code expected)
        #raise Exception("Not implemented yet")
        def startState(self): return problem.startState()
        def isGoal(self, state):return problem.isGoal(state)
        def succAndCost(self,state): 
            reductedActionAndCost = []
            for (action, destination, cost) in problem.succAndCost(state):
                reductedActionAndCost.append((action, destination, cost+heuristic(destination)-heuristic(state)))
                #DEBUG_ONLY
               # (x,y,z) = state
               # print("newstate:",destination,"cost:",cost,"h(cur):",heuristic(state),"h(des):",heuristic(destination))

            return reductedActionAndCost

        # END_YOUR_CODE
    newProblem = NewSearchProblem()
    return newProblem

# Implements A-star search by doing a reduction.
class AStarSearch(util.SearchAlgorithm):
    def __init__(self, heuristic):
        self.heuristic = heuristic

    def solve(self, problem):
        # Reduce the |problem| to |newProblem|, which is solved by UCS.
        newProblem = astarReduction(problem, self.heuristic)
        #8**************************
        algorithm = util.UniformCostSearch()
        algorithm.solve(newProblem)

        # Copy solution back
        self.actions = algorithm.actions
        if algorithm.totalCost != None:
            self.totalCost = algorithm.totalCost + self.heuristic(problem.startState())
        else:
            self.totalCost = None
        self.numStatesExplored = algorithm.numStatesExplored

############################################################
# Problem 2b: Delivery

class DeliveryProblem(util.SearchProblem):
    # |scenario|: delivery specification.
    def __init__(self, scenario):
        self.scenario = scenario

    # Return the start state.
    def startState(self):
        # BEGIN_YOUR_CODE (around 1 line of code expected)
        #raise Exception("Not implemented yet")
        return (self.scenario.truckLocation, frozenset(), frozenset()) 
        # END_YOUR_CODE

    # Return whether |state| is a goal state or not.
    def isGoal(self, state):
        # BEGIN_YOUR_CODE (around 2 lines of code expected)
        #raise Exception("Not implemented yet")
        (pos, a, deliveredPack) = state
        return pos == self.scenario.truckLocation and len(deliveredPack) == len(self.scenario.pickupLocations)
        # END_YOUR_CODE

    # Return a list of (action, newState, cost) tuples corresponding to edges
    # coming out of |state|.
    def succAndCost(self, state):
        # Hint: Call self.scenario.getNeighbors((x,y)) to get the valid neighbors
        # at that location. In order for the simulation code to work, please use
        # the exact strings 'Pickup' and 'Dropoff' for those two actions.
        # BEGIN_YOUR_CODE (around 18 lines of code expected)
        #raise Exception("Not implemented yet")
        (loc, pickedUpPack, deliveredPack)=state
        ret= map(lambda (actionName, newLoc):(actionName, (newLoc, pickedUpPack, deliveredPack), 1+len(pickedUpPack)), self.scenario.getNeighbors(loc))
        if (loc in self.scenario.pickupLocations \
            and self.scenario.pickupLocations.index(loc) not in pickedUpPack\
            and self.scenario.pickupLocations.index(loc) not in deliveredPack):
            newPackage = self.scenario.pickupLocations.index(loc)
            newPackageList = pickedUpPack.union([newPackage])
            state_new = ("Pickup", (loc, newPackageList ,deliveredPack), 0)         
            ret.append(  state_new )
        if (loc in self.scenario.dropoffLocations and self.scenario.dropoffLocations.index(loc) in pickedUpPack):
            dropPack = self.scenario.dropoffLocations.index(loc)
            ret.append(   ("Dropoff", (loc, pickedUpPack.difference([dropPack]), deliveredPack.union([dropPack])), 0 )     )
        return ret

        # END_YOUR_CODE

############################################################
# Problem 2c: heuristic 1


# Return a heuristic corresponding to solving a relaxed problem
# where you can ignore all barriers and not do any deliveries,
# you just need to go home
def createHeuristic1(scenario):
    def heuristic(state):
        # BEGIN_YOUR_CODE (around 2 lines of code expected)
        #raise Exception("Not implemented yet")
        (loc, pickedUpPack,deliveredPack)=state
        goalPos = scenario.truckLocation
        return (abs(loc[0]-goalPos[0])+abs(loc[1]-goalPos[1]))
        # END_YOUR_CODE
    return heuristic

############################################################
# Problem 2d: heuristic 2

# Return a heuristic corresponding to solving a relaxed problem
# where you can ignore all barriers, but
# you'll need to deliver the given |package|, and then go home
def createHeuristic2(scenario, package):
    def heuristic(state):
        # BEGIN_YOUR_CODE (around 11 lines of code expected)
        #raise Exception("Not implemented yet")
        (loc, pickedUpPack, deliveredPack) = state
        goalPos = scenario.truckLocation
        weight = 1
        pickLoc = scenario.pickupLocations[package]
        dropLoc = scenario.dropoffLocations[package]
        # distance from loc to pickup location
        dis1 = abs(loc[0]-pickLoc[0]) + abs(loc[1]-pickLoc[1])
        #distance from pickup location to dropoff location
        dis2 = abs(pickLoc[0]-dropLoc[0])+abs(pickLoc[1]-dropLoc[1])
        #distance from dropoff location to goal 
        dis3 = abs(dropLoc[0]-goalPos[0])+ abs(dropLoc[1]-goalPos[1])
        #distance from loc to dropoff location
        dis4 = abs(loc[0]-dropLoc[0])+abs(loc[1]-dropLoc[1])
        #distance from loc to goal
        dis5 = abs(loc[0]-goalPos[0])+abs(loc[1]-goalPos[1])
        if (package not in pickedUpPack and package not in deliveredPack):
            return weight*dis1 + (weight+1)*dis2+ weight*dis3
        elif(package in pickedUpPack and package not in deliveredPack):
            return (weight+1)*dis4 + weight*dis3
        else:
            return weight*dis5
            

        # END_YOUR_CODE
    return heuristic

############################################################
# Problem 2e: heuristic 3

# Return a heuristic corresponding to solving a relaxed problem
# where you will delivery the worst(i.e. most costly) |package|,
# you can ignore all barriers.
# Hint: you might find it useful to call
# createHeuristic2.
def createHeuristic3(scenario):
    # BEGIN_YOUR_CODE (around 5 lines of code expected)
    #raise Exception("Not implemented yet")
    def heuristic(state):
        (loc, pickedUpPack, deliveredPack) = state
        hmax = createHeuristic1(scenario)(state)
        for x in range(0, len(scenario.pickupLocations)):
            if (createHeuristic2(scenario, x)(state)>hmax): 
                hmax = createHeuristic2(scenario, x)(state)
        return hmax
    return heuristic
    # END_YOUR_CODE
