import math


def solveQuad(a, b, c):
    """
    Solve a quadratic equation.  Returns a list of solutions from length 0 to 2
    :param a:
    :param b:
    :param c:
    :return:
    """
    discriminant = (b ** 2) - (4 * a * c)
    divisor = 2 * a
    if discriminant < 0.0:
        return []
    elif divisor == 0.0:
        if b == 0.0:
            return []
        else:
            return [-c / b]
    elif discriminant > 0.0:
        sdiscriminant = math.sqrt(discriminant)
        return [(-b - sdiscriminant) / divisor,
                (-b + sdiscriminant) / divisor]
    else:
        return [-b / divisor]
