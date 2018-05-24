#include <math.h>
#include <Python.h>
#include "geometry.h"
#include "numpy/arrayobject.h"

//Store a numpy vector into a point data structure
void fromPyVec(Vec2* vec,PyObject *pyVec)
{
	double *data = (double *)PyArray_DATA(pyVec);
	vec->x = data[0];
	vec->y = data[1];
}

//Build an array of LineSegments from a list of LineSegment python objects
void fromPyLineSegment(LineSegment* line,PyObject *pyLine)
{
	PyObject* pyP1 = PyObject_GetAttrString(pyLine, "p1");
	PyObject* pyP2 = PyObject_GetAttrString(pyLine, "p2");
	PyObject* pyVelocity = PyObject_GetAttrString(pyLine, "velocity");


	fromPyVec(&line->p1,pyP1);
	fromPyVec(&line->p2,pyP2);
	fromPyVec(&line->velocity,pyVelocity);
	if(PyObject_HasAttrString(pyLine,"n"))
	{
    	PyObject* norm = PyObject_GetAttrString(pyLine, "n");
    	fromPyVec(&line->norm,norm);
	}
	else
	{
		line->norm.x=0;
		line->norm.y=0;
	}
	if(PyObject_HasAttrString(pyLine,"invTan"))
	{
    	PyObject* invTan = PyObject_GetAttrString(pyLine, "invTan");
    	fromPyVec(&line->invTan,invTan);
	}
	else
	{
		line->invTan.x=0;
		line->invTan.y=0;
	}
}
//Build an array of LineSegments from a list of LineSegment python objects
LineSegment* buildLineSegments(PyObject *lineList)
{
    Py_ssize_t length = PySequence_Length(lineList);
    LineSegment* lines = malloc(length*sizeof(LineSegment));
    LineSegment* line;
    int i;
	PyObject *pyLine;
    for (i=0; i<length; i++)
    {
    	line = lines+i;
    	pyLine = PyList_GetItem(lineList, i);
    	fromPyLineSegment(line,pyLine);
    }
    return lines;
}

void addTo(Vec2* v1, Vec2* v2)
{
	v1->x += v2->x;
	v1->y += v2->y;
}
void subBy(Vec2* v1, Vec2* v2)
{
	v1->x -= v2->x;
	v1->y -= v2->y;
}

void add(Vec2* result, Vec2* v1, Vec2* v2)
{
	result->x = v1->x + v2->x;
	result->y = v1->y + v2->y;
}

void sub(Vec2* result, Vec2* v1, Vec2* v2)
{
	result->x = v1->x - v2->x;
	result->y = v1->y - v2->y;
}
void scale(Vec2* result, Vec2* vec, double scaleFactor)
{
	result->x = vec->x * scaleFactor;
	result->y = vec->y * scaleFactor;
}
double dot(Vec2* v1, Vec2* v2)
{
	return v1->x * v2->x + v1->y * v2->y;
}
double magSquared(Vec2* vec)
{
	return vec->x*vec->x+vec->y*vec->y;
}
double mag(Vec2* vec)
{
	return sqrt(vec->x*vec->x+vec->y*vec->y);
}
double safeQuotient(double numerator,double divisor,double zeroValue)
{
	if(divisor == 0.0)
	{
		return zeroValue;
	}
	else
	{
		return numerator / divisor;
	}
}
/**
    Determine if the straight path, from pathStart to pathEnd, intersects this one-sided line segment.  Velocity is ignored and
    should have already been accounted for when choosing pathEnd in the coordinate frame of the moving line.

    If end-points of the path or the line-segment are overlapping, then it IS considered an intersection.  However,
    we don't care about this distinction and it may be masked by higher-level functions.

    If the velocity from pathStart to pathEnd is parallel or in the direction of the line segment normal,
    then it is NOT considered an intersection (only an intersection if this will cause the vehicle to go deeper into the
    obstacle).
 */
int intersectPathAndStaticLineC(LineSegment *lineSegment, Vec2 *pathStart, Vec2 *pathEnd)
{
	Vec2 p1diff;
	Vec2 p2diff;
	Vec2 pathTan;
	double pathTanNormal;
	double normalDistanceP1;
	double normalDistanceP2;
	double t;
	double tanValue;


	//p1diff = pathStart - lineSegment.p1;
	sub(&p1diff,pathStart,&lineSegment->p1);

    //Distance to one-sided line segment, from pathStart, in the direction of the normal.
	normalDistanceP1 = dot(&lineSegment->norm, &p1diff);

    //If this is negative then pathStart is "behind" the one-sided line-segment and no intersection is possible.
    if(normalDistanceP1 < 0.0)
        return 0;

    //Distance to one-sided line segment, from pathEnd, in the direction of the normal.

    sub(&p2diff,pathEnd,&lineSegment->p1);
    //p2diff = pathEnd - self.p1

    normalDistanceP2 = dot(&lineSegment->norm, &p2diff);

    //If this is positive then pathEnd is "in front of" the one-sided line-segment and no intersection is possible.
    if(normalDistanceP2 > 0.0)
        return 0;

    //The path's tangent vector (not a unit vector)
    sub(&pathTan, pathEnd, pathStart);
    //pathTan = pathEnd - pathStart

    //Equivalent of: pathTanNormal = dot(n,pathTan).  We don't normalize as the magnitude of this cancels out with itself later.
    //pathTanNormal = normalDistanceP2 - normalDistanceP1
    pathTanNormal = normalDistanceP2 - normalDistanceP1;

//    As you move from pathStart towards pathEnd you approach intersection with the infinite line defined by the one-sided
//    line segment. The crossing point will be at pathStart + pathTan * t, for some t.
//    t is just the ratio below:


    if(pathTanNormal == 0.0)
    {
    	//This will be 0.0 if pathStart and pathEnd are the same.
    	//We don't consider this a collision as it means the vehicle is not moving in coordinate frame of the line.
    	return 0;
    }
    t = -normalDistanceP1 / pathTanNormal;
    //While moving towards the line perpendicularly, we also moved along the line tangentially.
    //We don't care how far we moved in absolute space, only in "tangent-unit-space".  In this space p1 is 0, p2 is 1.

    tanValue = (p1diff.x + pathTan.x * t) * lineSegment->invTan.x +
    		   (p1diff.y + pathTan.y * t) * lineSegment->invTan.y;

    //tanValue = dot(p1diff + pathTan * t, lineSegment->invTan);
    return tanValue >= 0.0 && tanValue <= 1.0;
}


int intersectPathAndLineC(LineSegment* lineSegment, double startTime, Vec2* pathStart, Vec2* pathEnd, double speed)
{
	Vec2 pathVector;
	Vec2 velocity;
	Vec2 newPathEnd;
	Vec2 offset;
	Vec2 newPathStart;

	double distance;
	double t;

//    direction = endPoint - startPoint
//    distance = np.linalg.norm(direction)
//    if distance == 0.0:
//        return False

	sub(&pathVector,pathEnd,pathStart);
    distance = mag(&pathVector);
    if(distance == 0.0)
        return 0;

//	# velocity vector - has magnitude in speed heading in velocity from start to end
//	velocity = (speed / distance) * direction
//
//	# Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
//	velocity -= self.velocity
	scale(&velocity, &pathVector, speed / distance);
    subBy(&velocity,&lineSegment->velocity);

//  Time to get from pathStart to end
//	t = distance / speed
//
//	# The new end point takes the same time to reach, but at a new offset heading
//	endPoint = startPoint + velocity * t

    if(speed == 0.0)
    {
    	//If moving infinitely slowly, the end of the path is the start, because the line, will have moved an infinite distance, while the
    	//vehicle stood still.
    	newPathEnd = *pathStart;
    }
    else
    {
        t = distance / speed;

        //The new end point (in the coordinate frame of the moving line) takes the same time to reach, but at a new offset heading
        //newPathEnd = pathStart + velocity * t
        newPathEnd.x = pathStart->x + velocity.x * t;
        newPathEnd.y = pathStart->y + velocity.y * t;
    }


//    offset = -self.velocity * startTime
//    return self.checkLineIntersection(startPoint + offset, endPoint + offset)

    //Given the start time, this line will have moved.  We offset the pathStart and pathEnd in the opposite direction
    scale(&offset,&lineSegment->velocity,-startTime);
    add(&newPathStart,pathStart,&offset);
    addTo(&newPathEnd,&offset);
    return intersectPathAndStaticLineC(lineSegment,&newPathStart,&newPathEnd);
}
void printVec(Vec2 *vec)
{
	printf("(%f,%f)",vec->x,vec->y);
}
void printLine(LineSegment *lineSegment)
{
	printf("P1: ");
	printVec(&lineSegment->p1);
	printf("\n");
	printf("P2: ");
	printVec(&lineSegment->p2);
	printf("\n");
	printf("Velocity: ");
	printVec(&lineSegment->velocity);
	printf("\n");
	printf("Norm: ");
	printVec(&lineSegment->norm);
	printf("\n");
	printf("InvTan: ");
	printVec(&lineSegment->invTan);
	printf("\n");
}
