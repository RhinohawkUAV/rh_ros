#include <Python.h>
#include "geometry.h"
#include "numpy/arrayobject.h"
static PyObject* test_print(PyObject *self, PyObject *args)
{
    const char *string;

    if (!PyArg_ParseTuple(args, "s", &string))
        return NULL;
    printf("%s",string);
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* createIntersectionDetector(PyObject *self, PyObject *args)
{
	PyObject *lineList,*circleList;
    if (!PyArg_ParseTuple(args, "O!O!", &PyList_Type, &lineList,&PyList_Type, &circleList))
        return NULL;
    Py_ssize_t numLineSegments = PySequence_Length(lineList);
    Py_ssize_t numCircles = PySequence_Length(circleList);
    LineSegment* lineSegments = buildLineSegments(lineList);
    Circle* circles = buildCircles(circleList);
    ObstacleCourse* obstacleCourse = malloc(sizeof(ObstacleCourse));
    obstacleCourse->numLineSegments = (int)numLineSegments;
    obstacleCourse->lineSegments = lineSegments;
    obstacleCourse->numCircles = (int)numCircles;
    obstacleCourse->circles = circles;

    return PyLong_FromVoidPtr((void*)obstacleCourse);
}
static PyObject* testIntersectionDetector(PyObject *self, PyObject *args)
{
	int i;
	PyObject* handle;
	if(!PyArg_ParseTuple(args,"O",&handle))
		return NULL;
	ObstacleCourse* obstacleCourse = (ObstacleCourse*)PyLong_AsVoidPtr(handle);
	for(i=0;i<obstacleCourse->numLineSegments;i++)
	{
	    printLine(obstacleCourse->lineSegments+i);
	}

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* testIntersection(PyObject *self, PyObject *args)
{
	PyObject* handle;
	PyObject *pyPathStart,*pyVelocity;

    double startTime,pathTime;
    Vec2 pathStart,velocity;
	ObstacleCourse* obstacleCourse;

    if (!PyArg_ParseTuple(args, "OdOOd", &handle, &startTime, &pyPathStart, &pyVelocity, &pathTime))
        return NULL;
    fromPyVec(&pathStart,pyPathStart);
    fromPyVec(&velocity,pyVelocity);

	obstacleCourse = (ObstacleCourse*)PyLong_AsVoidPtr(handle);
	for(int i=0;i<obstacleCourse->numLineSegments;i++)
	{
	    if(testPathIntersectsLine(obstacleCourse->lineSegments+i, startTime, &pathStart, &velocity, pathTime))
	    {
	    	Py_RETURN_TRUE;
	    }
	}
	for(int i=0;i<obstacleCourse->numCircles;i++)
	{
	    if(testPathIntersectsCircle(obstacleCourse->circles+i, startTime, &pathStart, &velocity, pathTime))
	    {
	    	Py_RETURN_TRUE;
	    }
	}
	Py_RETURN_FALSE;
}
static PyMethodDef methods[] = {
	    {"test_print",  test_print, METH_VARARGS, "Print a string."},
	    {"createIntersectionDetector",  createIntersectionDetector, METH_VARARGS, "Create a C object representing all obstacles, which can be tested against."},
	    {"testIntersectionDetector",  testIntersectionDetector, METH_VARARGS, ""},
	    {"testIntersection",  testIntersection, METH_VARARGS, "Test if a given path intersects any of the paths stored by createIntersectionDetector."},
		{NULL, NULL, 0, NULL}        /* Sentinel */
};

//Must be named init<modname>
PyMODINIT_FUNC initfastPathIntersect(void)
{
    //Must be module's name
    (void) Py_InitModule("fastPathIntersect", methods);
}

