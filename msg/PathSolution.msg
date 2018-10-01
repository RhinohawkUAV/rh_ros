SolutionWaypoint[] solutionWaypoints
PathSegment[] solutionPathSegments

#0 - Failed: Explored entire search space and did not find a solution.  Most likely caused by starting inside an NFZ buffer
#			 or so close that dodging is impossible.  In the future, the inside buffer case needs to be handled by me.
#1 - Incomplete: Timed out searching for a solution.  Just use the result as if it were a complete solution and we'll
#				 try computing again
#2 - Solved: Found a solution, through all waypoints
#3 - Optimal: Path finder explored all solutions and is confident this is about the best you can do!
uint8 quality

#How many waypoints does the path successfully go through?
int64 numWaypointsCompleted

#How long will the path take to traverse?
float64 estimatedTime