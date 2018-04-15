#include <Python.h>

static PyObject *test_print(PyObject *self, PyObject *args)
{
    const char *string;

    if (!PyArg_ParseTuple(args, "s", &string))
        return NULL;
    printf("%s",string);
    Py_INCREF(Py_None);
    return Py_None;
}

static PyMethodDef methods[] = {
    {"test_print",  test_print, METH_VARARGS, "Print a string."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

//Must be named init<modname>
PyMODINIT_FUNC inittestmod(void)
{
    //Must be module's name
    (void) Py_InitModule("testmod", methods);
}

