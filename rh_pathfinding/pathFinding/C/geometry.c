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

//Build a LineSegment from a Python LineSegment object
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
void fromPyCircle(Circle* circle, PyObject* pyCircle)
{
	PyObject* pyCenter = PyObject_GetAttrString(pyCircle, "center");
	PyObject* pyVelocity = PyObject_GetAttrString(pyCircle, "velocity");
	PyObject* pyRadius = PyObject_GetAttrString(pyCircle, "radius");
	fromPyVec(&circle->center,pyCenter);
	fromPyVec(&circle->velocity,pyVelocity);
	double radius = PyFloat_AsDouble(pyRadius);
	circle->radiusSquared = radius*radius;
}

//Build an array of LineSegments from a list of LineSegment python objects
LineSegment* buildLineSegments(PyObject *lineList)
{
    Py_ssize_t numLineSegments = PySequence_Length(lineList);
    LineSegment* lines = NULL;
    if(numLineSegments > 0)
    {
    	lines = malloc(numLineSegments*sizeof(LineSegment));
        for (int i=0; i<numLineSegments; i++)
        {
        	fromPyLineSegment(lines+i,PyList_GetItem(lineList, i));
        }
    }
    return lines;
}
//Build an array of Circles from a list of CircleObstacle Python objects
Circle* buildCircles(PyObject *circleList)
{

    Py_ssize_t numCircles = PySequence_Length(circleList);
    Circle* circles = NULL;
    if(numCircles>0)
    {
    	circles = malloc(numCircles*sizeof(Circle));
        for (int i=0; i<numCircles; i++)
        {
        	fromPyCircle(circles+i,PyList_GetItem(circleList, i));
        }
    }
    return circles;
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
int testPathIntersectsStaticLine(LineSegment *lineSegment, Vec2 *pathStart, Vec2 *pathEnd)
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

int testPathIntersectsStaticCircle(Circle* circle, Vec2* pathStart, Vec2* pathVec)
{
	Vec2 fromCenter;
	sub(&fromCenter, pathStart, &circle->center);
	double a = dot(pathVec,pathVec);
	double b = 2.0 * dot(pathVec,&fromCenter);
	double c = dot(&fromCenter,&fromCenter) - circle->radiusSquared;
    double discriminant = b * b - (4 * a * c);
    double divisor = 2 * a;

    if(discriminant < 0.0)
        return 0;
    else if(divisor == 0.0)
    {
        if(b == 0.0)
			return 0;

        double solution = -c / b;
        return solution >= 0.0 && solution <= 1.0;
    }

    double sdiscriminant = sqrt(discriminant);
	double solution1 = (-b - sdiscriminant) / divisor;
	double solution2 = (-b + sdiscriminant) / divisor;
	return (solution1 >= 0.0 && solution1 <= 1.0) || (solution2 >= 0.0 && solution2 <= 1.0);
}

void adjustPathToObstacleVelocity(Vec2* newPathStart, Vec2* newPathVec, Vec2* obstacleVelocity, double startTime, Vec2* pathStart, Vec2* pathVelocity, double pathTime)
{
    //velocity = pathVelocity - lineSegment.velocity
	Vec2 velocity;
	sub(&velocity,pathVelocity,obstacleVelocity);

	scale(newPathVec,&velocity,pathTime);

	//offset = -self.velocity * startTime
	Vec2 offset;
	scale(&offset,obstacleVelocity,-startTime);

	add(newPathStart,pathStart,&offset);
}
int testPathIntersectsLine(LineSegment* lineSegment, double startTime, Vec2* pathStart, Vec2* pathVelocity, double pathTime)
{
	Vec2 newPathStart;
	Vec2 newPathVec;
	adjustPathToObstacleVelocity(&newPathStart,&newPathVec,&lineSegment->velocity,startTime, pathStart, pathVelocity, pathTime);
	Vec2 newPathEnd;
	add(&newPathEnd,&newPathStart,&newPathVec);
	return testPathIntersectsStaticLine(lineSegment,&newPathStart,&newPathEnd);
}



int testPathIntersectsCircle(Circle* circle, double startTime, Vec2* pathStart, Vec2* pathVelocity, double pathTime)
{
	Vec2 newPathStart;
	Vec2 newPathVec;
	adjustPathToObstacleVelocity(&newPathStart,&newPathVec,&circle->velocity,startTime, pathStart, pathVelocity, pathTime);
	return testPathIntersectsStaticCircle(circle,&newPathStart,&newPathVec);
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
