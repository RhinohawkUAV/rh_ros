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


void fromPyVec(Vec2* vec,PyObject *pyVec);
void fromPyLineSegment(LineSegment* line,PyObject *pyLine);
LineSegment* buildLineSegments(PyObject *lineList);
void add(Vec2* result, Vec2* v1, Vec2* v2);
void sub(Vec2* result, Vec2* v1, Vec2* v2);
double dot(Vec2* v1, Vec2* v2);
void printVec(Vec2 *vec);
void printLine(LineSegment *lineSegment);
int intersectPathAndLineC(LineSegment* lineSegment, double startTime, Vec2* pathStart, Vec2* pathEnd, double speed);
int intersectPathAndStaticLineC(LineSegment *lineSegment, Vec2 *start, Vec2 *end);
