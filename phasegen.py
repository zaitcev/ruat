#!/usr/bin/python3
#
# The generator phase lookup tables
#

import math
import sys

TAG="phasegen"

class ParamError(Exception):
    pass

class Param:
    def __init__(self, argv):
        skip = 1;  # Do skip=1 for full argv.
        #: Output name, stdout if not given
        self.outname = None
        for i in range(len(argv)):
            if skip:
                skip = 0
                continue
            arg = argv[i]
            if len(arg) != 0 and arg[0] == '-':
                if arg == "-o":
                    if i+1 == len(argv):
                        raise ParamError("Parameter -o needs an argument")
                    self.outname = argv[i+1]
                    skip = 1;
                else:
                    raise ParamError("Unknown parameter " + arg)
            else:
                raise ParamError("Positional parameter supplied")


def main(args):
    try:
        par = Param(args)
    except ParamError as e:
        print(TAG+": %s" % e, file=sys.stderr)
        print("Usage:", TAG+" [-o outfile]", file=sys.stderr)
        return 1

    if par.outname:
        outfp = open(par.outname, 'w')
    else:
        outfp = sys.stdout

    #
    # Compute the compound table from a 12 bits signed integer to
    # 8 bits signed integer.
    #
    # We're too lazy to compute the 70.0, so we just iterated for the
    # compounding to be linear near zero (70.0 ~~ com_fac).
    def com_fun(x):
        return math.log(1.0 + float(x / 70.0))

    com_fac = 255.0 / com_fun(2047)
    print("// com_fac == %f" % (com_fac,), file=outfp)

    print("int com_tab[2048] = {", file=outfp)
    print("    0,", file=outfp)
    old_cy = 0
    crev = [0]*256;
    for i in range(1, 2048):
        cy = int(com_fun(i) * com_fac)
        if cy != old_cy:
            crev[cy] = i
            print("  %3d, // %4d [%d]" % (cy, i, cy), file=outfp)
            old_cy = cy
        else:
            print("  %3d, // %4d" % (cy, i), file=outfp)
    print("};", file=outfp)

    #
    # Compute the arc-tangent table
    #
    print("float phi_tab[256][256] = {", file=outfp)
    for i in range(256):
        i_raw = crev[i]
        print("  {", file=outfp)
        for j in range(256):
            j_raw = crev[j]
            if i == 0 and j == 0:
                phi = 0.0
            else:
                phi = math.atan2(float(i_raw), float(j_raw))
            print("    %f%s" % (phi, ("" if j == 255 else ",")), file=outfp)
        print("  }" if i == 255 else "  },", file=outfp)
    print("};", file=outfp)

    if par.outname:
        outfp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
