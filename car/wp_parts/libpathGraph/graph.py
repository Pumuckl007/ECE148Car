from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import heapq

class Graph():
    def __init__(self):
        self.nodes = []
        self.adjecencyList = []
        self.edgeThreshold = 1

    def add_node(self, point):
        point = self.point_to_rads(point)
        idx = len(self.nodes)
        self.nodes.append(point)
        self.adjecencyList.append([])
        return idx

    def append_node(self, point, connectionIdx):
        point = self.point_to_rads(point)
        idx = len(self.nodes)
        self.nodes.append(point)
        self.adjecencyList.append([connectionIdx])
        self.adjecencyList[connectionIdx].append(idx)
        return idx

    def find_path(self, currLocation, goalLocation):
        currLocation = self.point_to_rads(currLocation)
        goalLocation = self.point_to_rads(goalLocation)
        startIdx = self.find_closest_vertex(currLocation)
        endIdx = self.find_closest_vertex(goalLocation)
        return self.dijkstra(startIdx, endIdx)

    def dijkstra(self, start, end):
        prev = []
        dist = []
        q = []
        for nodeIdx in range(0, len(self.nodes)):
            q.append([100000000, -1, nodeIdx, False])

        q[start][0] = 0

        while q :
            min = self.findMin(q)
            idx = min[2]
            node = self.nodes[idx]
            adjecencyList = self.adjecencyList[idx]
            for neighbor in adjecencyList:
                if neighbor == end:
                    q[neighbor][1] = idx
                    return self.build_path(q, end)
                otherNode = self.nodes[neighbor]
                newDist = min[0] + self.dist_between_gps_points(node, otherNode)
                if newDist < q[neighbor][0]:
                    q[neighbor][0] = newDist
                    q[neighbor][1] = idx

        return []

    def findMin(self, q):
        min = q[0]
        minIdx = 0
        for i in range(1, len(q)):
            if q[i][3] :
                continue
            if q[i][0] < min[0]:
                min = q[i]
                minIdx = i
        q[minIdx][3] = True
        return min

    def build_path(self, prev, end):
        path = [end]
        prevIdx = prev[end][1]
        while not prevIdx == -1:
            path.append(prevIdx)
            prevIdx = prev[prevIdx][1]

        path.reverse()
        pointPath = []
        for idx in path:
            pointPath.append(self.point_to_dgr(self.nodes[idx]))
        return pointPath

    def find_closest_vertex(self, currLocation):
        minDist = self.dist_between_gps_points(currLocation, self.nodes[0])
        index = 0
        for i in range(1, len(self.nodes)):
            dist = self.dist_between_gps_points(currLocation, self.nodes[i])
            if dist < minDist :
                index = i
                minDist = dist
        return index


    def dist_between_gps_points(self, pointA, pointB):
        """
        Method to calculate the straight-line approximation between two gps coordinates.
        Used for distances on the 10-1000m scale.

        @params: two gps points A & B (radians) defined by lat and long coordinates
        @return: distance between the two points in meters
        """

        # radius of earth (m)
        r_earth = 6371e3

        # extract lat and long coordinates
        lat1 = pointA[0]
        lon1 = pointA[1]
        lat2 = pointB[0]
        lon2 = pointB[1]

        dlat = lat2 - lat1  # change in latitude
        dlon = lon2 - lon1  # change in longitude

        dx = r_earth * dlon * cos((lat1+lat2)/2)
        dy = r_earth * dlat

        dist = sqrt(square(dx)+square(dy))  # straight line approximation

        return dist

    def point_to_rads(self, point):
        return point[0]*pi/180, point[1]*pi/180

    def point_to_dgr(self, point):
        return point[0]*180/pi, point[1]*180/pi
