"""
Python implementation of Andrew Walker's Dubins-Curves library.

Copyright (c) 2024 Rhys Mainwaring
"""

"""
Copyright (c) 2008-2018, Andrew Walker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import math

from enum import IntEnum


class DubinsPathType(IntEnum):
    LSL = 0  # Left straight left
    LSR = 1  # Left straight right
    RSL = 2  # Right straight left
    RSR = 3  # Right straight right
    RLR = 4  # Right left right
    LRL = 5  # Left right left

    def __str__(self):
        if self.value == DubinsPathType.LSL.value:
            return "LSL"
        elif self.value == DubinsPathType.LSR.value:
            return "LSR"
        elif self.value == DubinsPathType.RSL.value:
            return "RSL"
        elif self.value == DubinsPathType.RSR.value:
            return "RSR"
        elif self.value == DubinsPathType.RLR.value:
            return "RLR"
        elif self.value == DubinsPathType.LRL.value:
            return "LRL"
        else:
            return "INVALID"


class DubinsPath:
    def __init__(self):
        # the initial configuration
        self.qi = [0.0] * 3
        # the lengths of the three segments
        self.param = [0.0] * 3
        # model forward velocity / model angular velocity
        self.rho = 0.0
        # the path type described
        self.type = None  # DubinsPathType


class DubinsErrorType(IntEnum):
    EDUBOK = 0  # No error
    EDUBCOCONFIGS = 1  # Colocated configurations
    EDUBPARAM = 2  # Path parameterisitation error
    EDUBBADRHO = 3  # the rho value is invalid
    EDUBNOPATH = 4  # no connection between configurations with this word

    def __str__(self):
        if self.value == DubinsErrorType.EDUBOK.value:
            return "OK"
        elif self.value == DubinsErrorType.EDUBCOCONFIGS.value:
            return "COLOCATED CONFIGS"
        elif self.value == DubinsErrorType.EDUBPARAM.value:
            return "PATH PARAMETERISATION ERROR"
        elif self.value == DubinsErrorType.EDUBBADRHO.value:
            return "INVALID RHO"
        elif self.value == DubinsErrorType.EDUBNOPATH.value:
            return "NO CONNECTION BETWEEN CONFIGURATIONS"
        else:
            return "INVALID"


# from dubins.c
EPSILON = 10.0e-10
INFINITY = math.inf


class SegmentType(IntEnum):
    L_SEG = 0
    S_SEG = 1
    R_SEG = 2

    def __str__(self):
        if self.value == SegmentType.L_SEG.value:
            return "L_SEG"
        elif self.value == SegmentType.S_SEG.value:
            return "S_SEG"
        elif self.value == SegmentType.R_SEG.value:
            return "R_SEG"
        else:
            return "INVALID"


# The segment types for each of the Path types
DIRDATA = [
    [SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.L_SEG],
    [SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.R_SEG],
    [SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.L_SEG],
    [SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.R_SEG],
    [SegmentType.R_SEG, SegmentType.L_SEG, SegmentType.R_SEG],
    [SegmentType.L_SEG, SegmentType.R_SEG, SegmentType.L_SEG],
]


class DubinsIntermediateResults:
    def __init__(self):
        self.alpha = 0.0
        self.beta = 0.0
        self.d = 0.0
        self.sa = 0.0
        self.sb = 0.0
        self.ca = 0.0
        self.cb = 0.0
        self.c_ab = 0.0
        self.d_sq = 0.0


# double fmodr( double x, double y)
def fmodr(x, y):
    """
    Floating point modulus suitable for rings

    fmod doesn't behave correctly for angular quantities, this function does
    """
    return x - y * math.floor(x / y)


# double mod2pi( double theta )
def mod2pi(theta):
    return fmodr(theta, 2 * math.pi)


def DubinsPathSamplingCallback(q, t, user_data):
    """
    Callback function for path sampling

    @note the q parameter is a configuration
    @note the t parameter is the distance along the path
    @note the user_data parameter is forwarded from the caller
    @note return non-zero to denote sampling should be stopped
    """
    return DubinsErrorType.EDUBOK


# int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho)
def dubins_shortest_path(q0, q1, rho):
    """
    Generate a path from an initial configuration to
    a target configuration, with a specified maximum turning
    radii

    A configuration is (x, y, theta), where theta is in radians, with zero
    along the line x = 0, and counter-clockwise is positive

    @param path  - the resultant path
    @param q0    - a configuration specified as an array of x, y, theta
    @param q1    - a configuration specified as an array of x, y, theta
    @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
    @return      - (non-zero on error, DubinsPath)
    """
    path = DubinsPath()
    i = 0
    errcode = 0
    cost = 0.0
    best_cost = INFINITY
    best_word = -1
    # in_ = DubinsIntermediateResults()
    errcode, in_ = dubins_intermediate_results(q0, q1, rho)
    if errcode != DubinsErrorType.EDUBOK:
        return errcode, path

    path.qi[0] = q0[0]
    path.qi[1] = q0[1]
    path.qi[2] = q0[2]
    path.rho = rho

    for i in range(6):
        pathType = DubinsPathType(i)
        # params = [0.0] * 3
        errcode, params = dubins_word(in_, pathType)
        if errcode == DubinsErrorType.EDUBOK:
            cost = params[0] + params[1] + params[2]
            if cost < best_cost:
                best_word = i
                best_cost = cost
                path.param[0] = params[0]
                path.param[1] = params[1]
                path.param[2] = params[2]
                path.type = pathType
    if best_word == -1:
        return DubinsErrorType.EDUBNOPATH, path
    return DubinsErrorType.EDUBOK, path


# int dubins_path(DubinsPath* path, double q0[3], double q1[3], double rho, DubinsPathType pathType)
def dubins_path(q0, q1, rho, pathType):
    """
    Generate a path with a specified word from an initial configuration to
    a target configuration, with a specified turning radius

    @param path     - the resultant path
    @param q0       - a configuration specified as an array of x, y, theta
    @param q1       - a configuration specified as an array of x, y, theta
    @param rho      - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
    @param pathType - the specific path type to use
    @return         - non-zero on error
    """
    path = DubinsPath()
    errcode = 0
    # in_ = DubinsIntermediateResults()
    errcode, in_ = dubins_intermediate_results(q0, q1, rho)
    if errcode == DubinsErrorType.EDUBOK:
        # params = [0.0] * 3
        errcode, params = dubins_word(in_, pathType)
        if errcode == DubinsErrorType.EDUBOK:
            path.param[0] = params[0]
            path.param[1] = params[1]
            path.param[2] = params[2]
            path.qi[0] = q0[0]
            path.qi[1] = q0[1]
            path.qi[2] = q0[2]
            path.rho = rho
            path.type = pathType
    return errcode, path


# double dubins_path_length( DubinsPath* path )
def dubins_path_length(path):
    """
    Calculate the length of an initialised path

    @param path - the path to find the length of
    """
    length = 0.0
    length += path.param[0]
    length += path.param[1]
    length += path.param[2]
    length = length * path.rho
    return length


# double dubins_segment_length( DubinsPath* path, int i )
def dubins_segment_length(path, i):
    """
    Return the length of a specific segment in an initialized path

    @param path - the path to find the length of
    @param i    - the segment you to get the length of (0-2)
    """
    if (i < 0) or (i > 2):
        return INFINITY
    return path.param[i] * path.rho


# double dubins_segment_length_normalized( DubinsPath* path, int i )
def dubins_segment_length_normalized(path, i):
    """
    Return the normalized length of a specific segment in an initialized path

    @param path - the path to find the length of
    @param i    - the segment you to get the length of (0-2)
    """
    if (i < 0) or (i > 2):
        return INFINITY
    return path.param[i]


# DubinsPathType dubins_path_type( DubinsPath* path )
def dubins_path_type(path):
    """
    Extract an integer that represents which path type was used

    @param path    - an initialised path
    @return        - one of LSL, LSR, RSL, RSR, RLR or LRL
    """
    return path.type


# void dubins_segment( double t, double qi[3], double qt[3], SegmentType type)
def dubins_segment(t, qi, type):
    """
    @returns qt
    """
    qt = [0.0] * 3

    st = math.sin(qi[2])
    ct = math.cos(qi[2])
    if type == SegmentType.L_SEG:
        qt[0] = +math.sin(qi[2] + t) - st
        qt[1] = -math.cos(qi[2] + t) + ct
        qt[2] = t
    elif type == SegmentType.R_SEG:
        qt[0] = -math.sin(qi[2] - t) + st
        qt[1] = +math.cos(qi[2] - t) - ct
        qt[2] = -t
    elif type == SegmentType.S_SEG:
        qt[0] = ct * t
        qt[1] = st * t
        qt[2] = 0.0
    qt[0] += qi[0]
    qt[1] += qi[1]
    qt[2] += qi[2]

    return qt


# int dubins_path_sample( DubinsPath* path, double t, double q[3] )
def dubins_path_sample(path, t):
    """
    Calculate the configuration along the path, using the parameter t

    @param path - an initialised path
    @param t    - a length measure, where 0 <= t < dubins_path_length(path)
    @returns    - (retcode, q[3])
        retcode - non-zero if 't' is not in the correct range
        q - the configuration result
    """
    # the configuration result
    q = [0.0] * 3

    # tprime is the normalised variant of the parameter t
    tprime = t / path.rho
    qi = [0.0] * 3  # The translated initial configuration
    q1 = [0.0] * 3  # end-of segment 1
    q2 = [0.0] * 3  # end-of segment 2
    types = DIRDATA[path.type]
    p1 = 0.0
    p2 = 0.0

    if t < 0 or t > dubins_path_length(path):
        return DubinsErrorType.EDUBPARAM, q

    # initial configuration
    qi[0] = 0.0
    qi[1] = 0.0
    qi[2] = path.qi[2]

    # generate the target configuration
    p1 = path.param[0]
    p2 = path.param[1]
    q1 = dubins_segment(p1, qi, types[0])
    q2 = dubins_segment(p2, q1, types[1])
    if tprime < p1:
        q = dubins_segment(tprime, qi, types[0])

    elif tprime < (p1 + p2):
        q = dubins_segment(tprime - p1, q1, types[1])
    else:
        q = dubins_segment(tprime - p1 - p2, q2, types[2])

    # scale the target configuration, translate back to the original starting point
    q[0] = q[0] * path.rho + path.qi[0]
    q[1] = q[1] * path.rho + path.qi[1]
    q[2] = mod2pi(q[2])

    return DubinsErrorType.EDUBOK, q


# int dubins_path_sample_many(DubinsPath* path, double stepSize,
#                             DubinsPathSamplingCallback cb, void* user_data)
def dubins_path_sample_many(path, stepSize, cb, user_data):
    """
    Walk along the path at a fixed sampling interval, calling the
    callback function at each interval

    The sampling process continues until the whole path is sampled, or the callback returns a non-zero value

    @param path      - the path to sample
    @param stepSize  - the distance along the path for subsequent samples
    @param cb        - the callback function to call for each sample
    @param user_data - optional information to pass on to the callback

    @returns - EDUBOK on successful completion, or the result of the callback
    """
    retcode = 0
    x = 0.0
    length = dubins_path_length(path)
    while x < length:
        # q = [0.0] * 3
        result, q = dubins_path_sample(path, x)
        retcode = cb(q, x, user_data)
        if retcode != 0:
            return retcode
        x += stepSize
    return DubinsErrorType.EDUBOK


# int dubins_path_endpoint( DubinsPath* path, double q[3] )
def dubins_path_endpoint(path):
    """
    Convenience function to identify the endpoint of a path

    @param path - an initialised path

    @returns (retcode, q)
        retcode - zero on successful completion
        q - the configuration result

    """
    # q = [0.0] * 3
    result, q = dubins_path_sample(path, dubins_path_length(path) - EPSILON)
    return result, q


# int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
def dubins_extract_subpath(path, t):
    """
    Convenience function to extract a subset of a path

    @param path    - an initialised path
    @param t       - a length measure, where 0 < t < dubins_path_length(path)

    @returns       - (retcode, newpath)
        retcode - EDUBOK on success
        newpath - the resultant path
    """
    newpath = DubinsPath()

    # calculate the true parameter
    tprime = t / path.rho

    if (t < 0) or (t > dubins_path_length(path)):
        return DubinsErrorType.EDUBPARAM, newpath

    # copy most of the data
    newpath.qi[0] = path.qi[0]
    newpath.qi[1] = path.qi[1]
    newpath.qi[2] = path.qi[2]
    newpath.rho = path.rho
    newpath.type = path.type

    # fix the parameters
    newpath.param[0] = min(path.param[0], tprime)
    newpath.param[1] = min(path.param[1], tprime - newpath.param[0])
    newpath.param[2] = min(path.param[2], tprime - newpath.param[0] - newpath.param[1])
    return DubinsErrorType.EDUBOK, newpath


# int dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3], double q1[3], double rho)
def dubins_intermediate_results(q0, q1, rho):
    """
    @param q0 double[3] - not modified
    @param q1 double[3] - not modified
    @param rho double - not modified

    @returns (retcode, DubinsIntermediateResults)
    """
    in_ = DubinsIntermediateResults()

    dx = 0.0
    dy = 0.0
    D = 0.0
    d = 0.0
    theta = 0.0
    alpha = 0.0
    beta = 0.0
    if rho <= 0.0:
        return DubinsErrorType.EDUBBADRHO, in_

    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    D = math.sqrt(dx * dx + dy * dy)
    d = D / rho
    theta = 0

    # test required to prevent domain errors if dx=0 and dy=0
    if d > 0:
        theta = mod2pi(math.atan2(dy, dx))

    alpha = mod2pi(q0[2] - theta)
    beta = mod2pi(q1[2] - theta)

    in_.alpha = alpha
    in_.beta = beta
    in_.d = d
    in_.sa = math.sin(alpha)
    in_.sb = math.sin(beta)
    in_.ca = math.cos(alpha)
    in_.cb = math.cos(beta)
    in_.c_ab = math.cos(alpha - beta)
    in_.d_sq = d * d

    return DubinsErrorType.EDUBOK, in_


# int dubins_LSL(DubinsIntermediateResults* in, double out[3])
def dubins_LSL(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    tmp0 = 0.0
    tmp1 = 0.0
    p_sq = 0.0

    tmp0 = in_.d + in_.sa - in_.sb
    p_sq = 2 + in_.d_sq - (2 * in_.c_ab) + (2 * in_.d * (in_.sa - in_.sb))

    if p_sq >= 0:
        tmp1 = math.atan2((in_.cb - in_.ca), tmp0)
        out[0] = mod2pi(tmp1 - in_.alpha)
        out[1] = math.sqrt(p_sq)
        out[2] = mod2pi(in_.beta - tmp1)
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_RSR(DubinsIntermediateResults* in, double out[3])
def dubins_RSR(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    tmp0 = in_.d - in_.sa + in_.sb
    p_sq = 2 + in_.d_sq - (2 * in_.c_ab) + (2 * in_.d * (in_.sb - in_.sa))
    if p_sq >= 0:
        tmp1 = math.atan2((in_.ca - in_.cb), tmp0)
        out[0] = mod2pi(in_.alpha - tmp1)
        out[1] = math.sqrt(p_sq)
        out[2] = mod2pi(tmp1 - in_.beta)
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_LSR(DubinsIntermediateResults* in, double out[3])
def dubins_LSR(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    p_sq = -2 + (in_.d_sq) + (2 * in_.c_ab) + (2 * in_.d * (in_.sa + in_.sb))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2((-in_.ca - in_.cb), (in_.d + in_.sa + in_.sb)) - math.atan2(
            -2.0, p
        )
        out[0] = mod2pi(tmp0 - in_.alpha)
        out[1] = p
        out[2] = mod2pi(tmp0 - mod2pi(in_.beta))
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_RSL(DubinsIntermediateResults* in, double out[3])
def dubins_RSL(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    p_sq = -2 + in_.d_sq + (2 * in_.c_ab) - (2 * in_.d * (in_.sa + in_.sb))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2((in_.ca + in_.cb), (in_.d - in_.sa - in_.sb)) - math.atan2(
            2.0, p
        )
        out[0] = mod2pi(in_.alpha - tmp0)
        out[1] = p
        out[2] = mod2pi(in_.beta - tmp0)
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_RLR(DubinsIntermediateResults* in, double out[3])
def dubins_RLR(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    tmp0 = (6.0 - in_.d_sq + 2 * in_.c_ab + 2 * in_.d * (in_.sa - in_.sb)) / 8.0
    phi = math.atan2(in_.ca - in_.cb, in_.d - in_.sa + in_.sb)
    if math.fabs(tmp0) <= 1:
        p = mod2pi((2.0 * math.pi) - math.acos(tmp0))
        t = mod2pi(in_.alpha - phi + mod2pi(p / 2.0))
        out[0] = t
        out[1] = p
        out[2] = mod2pi(in_.alpha - in_.beta - t + mod2pi(p))
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_LRL(DubinsIntermediateResults* in, double out[3])
def dubins_LRL(in_):
    """
    @param in_ DubinsIntermediateResults - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3

    tmp0 = (6.0 - in_.d_sq + 2 * in_.c_ab + 2 * in_.d * (in_.sb - in_.sa)) / 8.0
    phi = math.atan2(in_.ca - in_.cb, in_.d + in_.sa - in_.sb)
    if math.fabs(tmp0) <= 1:
        p = mod2pi(2 * math.pi - math.acos(tmp0))
        t = mod2pi(-in_.alpha - phi + p / 2.0)
        out[0] = t
        out[1] = p
        out[2] = mod2pi(mod2pi(in_.beta) - in_.alpha - t + mod2pi(p))
        return DubinsErrorType.EDUBOK, out

    return DubinsErrorType.EDUBNOPATH, out


# int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, double out[3])
def dubins_word(in_, pathType):
    """
    @param in_ DubinsIntermediateResults - not modified
    @param pathType DubinsPathType - not modified

    @returns (retcode, out[3])
    """
    out = [0.0] * 3
    result = 0
    if pathType == DubinsPathType.LSL:
        result, out = dubins_LSL(in_)
    elif pathType == DubinsPathType.RSL:
        result, out = dubins_RSL(in_)
    elif pathType == DubinsPathType.LSR:
        result, out = dubins_LSR(in_)
    elif pathType == DubinsPathType.RSR:
        result, out = dubins_RSR(in_)
    elif pathType == DubinsPathType.LRL:
        result, out = dubins_LRL(in_)
    elif pathType == DubinsPathType.RLR:
        result, out = dubins_RLR(in_)
    else:
        result, out = DubinsErrorType.EDUBNOPATH, out

    return result, out
