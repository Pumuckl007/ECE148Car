
class Graph():
    def __init__(self):
        self.nodes = []
        self.adjecencyList = []
        self.edgeThreshold = 1

    def findNextPointToGoal(self, currLocation, goalLocation):
        if self.isOnEdge(currLocation):
            return self.findPathToGoal(currLocation, goalLocation)
        return self.findNearestPointOnEdge(currLocation)

    def isOnEdge(self, currLocation):
        for i in range(0, len(self.nodes)):
            for end in self.adjecencyList[i]:
                line = (self.nodes[i], self.nodes[end])
                dist = self.distanceLineToPoint(line, currLocation)

    def distanceLineToPoint(self, line, point):
        
