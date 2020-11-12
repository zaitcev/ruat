#!/usr/bin/python3
#
# pythagorean triangle generator
#
# The version in ruat does not deal any of the AirSpy-style unsigned
# representations: all integers are signed.
#

import math
import sys

TAG="pythagen"

# Param

class ParamError(Exception):
    pass

class Param:
    def __init__(self, argv):
        skip = 0;  # Do skip=1 for full argv.
        #: Number of bits between 12 and 8
        self.bits = 12
        for i in range(len(argv)):
            if skip:
                skip = 0
                continue
            arg = argv[i]
            if len(arg) != 0 and arg[0] == '-':
                if arg == "-b":
                    if i+1 == len(argv):
                        raise ParamError("Parameter -b needs an argument")
                    try:
                        self.bits = int(argv[i+1])
                    except ValueError:
                        raise ParamError("Invalid argument for -b (%s)" %
                                         argv[i+1])
                    skip = 1;
                else:
                    raise ParamError("Unknown parameter " + arg)
            else:
                raise ParamError("Positional parameter supplied")
        if not (7 <= self.bits <= 12):
            raise ParamError("Number of bits (-b) must be 7 .. 12")


def pyth(x, y):
    """
    :param x: an integer - not in the AirSpy format, just a normal integer
    :param y: ditto
    :returns: sqrt(x^2 + y^2), a float
    """
    return math.sqrt(float(x*x + y*y))

#def apyth(nbits, i, j):
#    """
#    :param nbits: number of bits in the unsigned integer (location of the zero)
#    :param i: an unsigned integer that is actually signed and offset by 2^nbits
#    :param j: ditto
#    """
#    return pyth(i - (1 << (nbits-1)), j - (1 << (nbits-1)))

def main(args):
    try:
        par = Param(args)
    except ParamError as e:
        print(TAG+": ", e, file=sys.stderr)
        print("Usage:", TAG+" [-b nbits] [-s]", file=sys.stderr)
        return 1

    #print("{")
    #n = 1 << par.bits
    #for i in range(n):
    #    print("  // %d" % i)
    #    for j in range(n):
    #        k = int(apyth(par.bits, i, j))
    #        comma = "" if i == n-1 and j == n-1 else ","
    #        print("  %d%s" % (k, comma))
    #print("}")

    # now the test
    smallest_fraction = 1.0
    smallest_i = None
    smallest_j = None
    smallest_pyth = None
    smallest_pyth0 = None
    largest_fraction = 1.0
    largest_i = None
    largest_j = None
    largest_pyth = None

    # 2048 is valid because -2048 exists (barely), but we don't test it.
    # The precision near the zero is absolutely awful when using less than 12.
    for i in range(100,2048):
        for j in range(100,2048):
            k0 = int(math.sqrt(float(i*i + j*j)))
            #k = int(apyth(par.bits, i+(1<<(par.bits-1)), j+(1<<(par.bits-1))))
            if par.bits != 12:
                i1 = i >> (12 - par.bits)
                j1 = j >> (12 - par.bits)
                k = int(pyth(i1, j1)) << (12 - par.bits)
            else:
                k = int(pyth(i, j))
            if k0 == 0:
                if k != 0:
                    f = 1000000.0
                else:
                    f = 1.0
            else:
                f = float(k)/float(k0)
            if f > largest_fraction:
                largest_fraction = f
                largest_i = i
                largest_j = j
                largest_pyth = k
            if f < smallest_fraction:
                smallest_fraction = f
                smallest_i = i
                smallest_j = j
                smallest_pyth = k
                smallest_pyth0 = k0

    print("smallest fraction %s pyth(%s,%s)=%s vs %s" % (
           smallest_fraction, smallest_i, smallest_j,
           smallest_pyth, smallest_pyth0))
    print("largest fraction %s pyth(%s,%s)=%s" % (
           largest_fraction, largest_i, largest_j, largest_pyth))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
