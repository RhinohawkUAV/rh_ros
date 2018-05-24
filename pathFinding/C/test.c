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
static PyObject* readList(PyObject *self, PyObject *args)
{
	PyObject *list;
    if (!PyArg_ParseTuple(args, "O!", &PyList_Type, &list))
        return NULL;
    LineSegment* lines = buildLineSegments(list);
    printLine(lines);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* intersectPathAndStaticLine(PyObject *self, PyObject *args)
{
	PyObject *pyLineSegment,*pyStart,*pyEnd;
    if (!PyArg_ParseTuple(args, "OOO", &pyLineSegment, &pyStart, &pyEnd))
        return NULL;
    Vec2 start,end;
    LineSegment line;
    fromPyLineSegment(&line,pyLineSegment);
    fromPyVec(&start,pyStart);
    fromPyVec(&end,pyEnd);
    int result = intersectPathAndStaticLineC(&line,&start,&end);
    return Py_BuildValue("i", result);
}
static PyObject* intersectPathAndLine(PyObject *self, PyObject *args)
{
    double startTime,speed;
	PyObject *pyLineSegment,*pyStart,*pyEnd;
    if (!PyArg_ParseTuple(args, "OdOOd", &pyLineSegment, &startTime, &pyStart, &pyEnd, &speed))
        return NULL;
    LineSegment line;
    Vec2 pathStart,pathEnd;
    fromPyLineSegment(&line,pyLineSegment);
    fromPyVec(&pathStart,pyStart);
    fromPyVec(&pathEnd,pyEnd);

    int result = intersectPathAndLineC(&line, startTime, &pathStart, &pathEnd, speed);

    return Py_BuildValue("i", result);
}



static PyMethodDef methods[] = {
	    {"test_print",  test_print, METH_VARARGS, "Print a string."},
	    {"readList",  readList, METH_VARARGS, "Reads a list of objects."},
	    {"intersectPathAndStaticLine",  intersectPathAndStaticLine, METH_VARARGS, "Check intersection between path and a static line."},
	    {"intersectPathAndLine",  intersectPathAndLine, METH_VARARGS, "Check intersection between path and a static line."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

//Must be named init<modname>
PyMODINIT_FUNC inittestmod(void)
{
    //Must be module's name
    (void) Py_InitModule("testmod", methods);
}

