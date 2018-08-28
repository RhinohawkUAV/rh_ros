#include <Python.h>
typedef struct Vec2Struct
{
	double x,y;
} Vec2;
typedef struct LineSegmentStruct
{
	Vec2 p1;
	Vec2 p2;
	Vec2 norm;
	Vec2 invTan;
	Vec2 velocity;
} LineSegment;

typedef struct CircleStruct
{
	Vec2 center;
	Vec2 velocity;
	double radiusSquared;
} Circle;

typedef struct ObstacleCourseStruct
{
	int numLineSegments;
	LineSegment* lineSegments;
	int numCircles;
	Circle* circles;
} ObstacleCourse;


void fromPyVec(Vec2* vec,PyObject *pyVec);
void fromPyLineSegment(LineSegment* line,PyObject *pyLine);
LineSegment* buildLineSegments(PyObject *lineList);
Circle* buildCircles(PyObject *circleList);
void add(Vec2* result, Vec2* v1, Vec2* v2);
void sub(Vec2* result, Vec2* v1, Vec2* v2);
double dot(Vec2* v1, Vec2* v2);
void printVec(Vec2 *vec);
void printLine(LineSegment *lineSegment);
int testPathIntersectsStaticLine(LineSegment *lineSegment, Vec2 *pathStart, Vec2 *pathEnd);
int testPathIntersectsLine(LineSegment* lineSegment, double startTime, Vec2* pathStart, Vec2* pathVelocity, double pathTime);
int testPathIntersectsStaticCircle(Circle* circle, Vec2* pathStart, Vec2* lineVector);
int testPathIntersectsCircle(Circle* circle, double startTime, Vec2* pathStart, Vec2* pathVelocity, double pathTime);

