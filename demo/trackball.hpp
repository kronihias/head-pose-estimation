#ifndef TRACKBALL_HPP_
#define TRACKBALL_HPP_

#include "vector.hpp"
#include "matrix.hpp"


#define TRACKBALLSIZE  ((float)1.0)

/*
 * simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */


template<class T>
static inline T tb_project_to_sphere(T r, T x, T y)
{
    float d, t, z;

    d = T( sqrt(x*x + y*y) );
    if (d < r * 0.70710678118654752440) {    /* Inside sphere */
        z = T( sqrt(r*r - d*d) );
    } else {           /* On hyperbola */
        t = T( r / 1.41421356237309504880 );
        z = T( t*t / d );
    }
    return z;
}

template<class T>
static inline math::matrix_fixed<T,3,3> trackball(T p1x, T p1y, T p2x, T p2y)
{
	math::matrix_fixed<T,3,3> rot;

    if (p1x == p2x && p1y == p2y) {
        /* Zero rotation */
        rot.set_identity();
    }
	else {
	    /*
	     * First, figure out z-coordinates for projection of P1 and P2 to
	     * deformed sphere
	     */
	     math_vector_3f vec1, vec2;
	     vec1[0] = p1x;
	     vec1[1] = p1y;
	     vec1[2] = tb_project_to_sphere(TRACKBALLSIZE,p1x,p1y);
	     vec2[0] = p2x;
	     vec2[1] = p2y;
	     vec2[2] = tb_project_to_sphere(TRACKBALLSIZE,p2x,p2y);

	    /*
	     *  Now, we want the cross product of P1 and P2
	     */
	    math_vector_3f c = cross_product(vec1,vec2);

	    /*
	     *  Figure out how much to rotate around that axis.
	     */
	    T t = (length(vec2-vec1) / (2.0*TRACKBALLSIZE));

	    /*
	     * Avoid problems with out-of-control values...
	     */
	    if (t > 1.0) t = 1.0;
	    if (t < -1.0) t = -1.0;
	    T phi = 2.0f * T( asin(t) );

	    rot = axis_to_rotation_matrix( phi, c);
	}
	return rot;
}

#endif /*TRACKBALL_HPP_*/
