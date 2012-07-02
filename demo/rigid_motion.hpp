#ifndef RIGID_MOTION_HPP_
#define RIGID_MOTION_HPP_

#include "common.hpp"
#include "vector.hpp"
#include "matrix.hpp"

#include <iostream>
#include <fstream>
#include <vector>

#define RADIANS 57.2957795

template <typename T>
class rigid_motion {

public:

	math::matrix_fixed<T,3,3> m_rotation;
	math::vector_fixed<T,3> m_translation;

	rigid_motion() {
		m_rotation.set_identity();
		m_translation.fill( T(0) );
	}

	void reset(){

		m_rotation.set_identity();
		m_translation.fill( T(0) );

	}

	rigid_motion(const math::vector_fixed<T,3> & _t) {
		m_rotation.set_identity();
		m_translation = _t;
	}

	rigid_motion(const math::matrix_fixed<T,3,3> & _rot) {
		m_rotation = _rot;
		m_translation.fill(T(0));
	}

	rigid_motion(const math::matrix_fixed<T,3,3> & _rot, const math::vector_fixed<T,3> & _t) {
		m_rotation = _rot;
		m_translation = _t;
	}

	rigid_motion(const T *  _rot, const T * _t) {
		m_rotation = math::matrix_fixed<T,3,3>(_rot);
		m_translation = math::vector_fixed<T,3>(_t);;
	}


	// apply rigid motion to point
	math::vector_fixed<T,3> apply( const math::vector_fixed<T,3> & _pt ) const {
		return m_rotation * _pt + m_translation;
	}


	math::vector_fixed<T,3> operator*( const math::vector_fixed<T,3> & _pt ) const {
		return m_rotation * _pt + m_translation;
	}


	bool read_file_txt(const char * _file) {
		std::ifstream stream;
		stream.open(_file);
		if(!stream.good()) {
			printf("Cannot read file: %s\n", _file);
			return false;
		}
		stream >> m_rotation;
		stream >> m_translation;
		stream.close();
		return true;
	}

	void write_file(const char * _file) const {
		std::ofstream stream;
		stream.open(_file);
		stream << m_rotation << std::endl;
		stream << m_translation << std::endl;
		stream.close();
	}


};

// concatenate rigid motion
template<class T>
static rigid_motion<T> operator*(const rigid_motion<T> & _rm1, const rigid_motion<T> & _rm2) {
	rigid_motion<T> rm_new;
	rm_new.m_rotation = _rm1.m_rotation * _rm2.m_rotation;
	rm_new.m_translation = _rm1.m_rotation * _rm2.m_translation + _rm1.m_translation;
	return rm_new;
}

// inverse rigid motion
template<class T>
static rigid_motion<T> inverse(const rigid_motion<T> & _rm) {
	rigid_motion<T> rm_new;
	rm_new.m_rotation = _rm.m_rotation.transpose();
	rm_new.m_translation = - rm_new.m_rotation * _rm.m_translation;
	return rm_new;
}

// Angle in radians, unlike OpenGL
// convert angle and direction to rotation matrix
template <class T>
static math::matrix_fixed<T,3,3> axis_to_rotation_matrix( const T & angle, const math::vector_fixed<T,3> & dir) {

	math::matrix_fixed<T,3,3> rot;

	T l = length(dir);
	if (l == T(0)) {
		rot.set_identity();
	}
	else {
		// normalize
		T l1 = T(1)/l;
		T x = dir[0]*l1;
		T y = dir[1]*l1;
		T z = dir[2]*l1;
		T s = sin(angle), c = cos(angle);
		T xs = x*s, ys = y*s, zs = z*s, c1 = T(1)-c;
		T xx = c1*x*x, yy = c1*y*y, zz = c1*z*z;
		T xy = c1*x*y, xz = c1*x*z, yz = c1*y*z;
		rot[0][0] = xx+c;
		rot[0][1] = xy-zs;
		rot[0][2] = xz+ys;
		rot[1][0] = xy+zs;
		rot[1][1] = yy+c;
		rot[1][2] = yz-xs;
		rot[2][0] = xz-ys;
		rot[2][1] = yz+xs;
		rot[2][2] = zz+c;
	}
	return rot;
}


// Angles in radians!
// convert euler angles to rotation matrix
template <class T>
static math::matrix_fixed<T,3,3> euler_to_rotation_matrix( const T & x, const T & y, const T & z) {

	math::matrix_fixed<T,3,3> rot;

	T A       = cos(x);
	T B       = sin(x);
	T C       = cos(y);
	T D       = sin(y);
	T E       = cos(z);
	T F       = sin(z);

	T AD      =   A * -D;
	T BD      =   B * -D;

	rot[0][0]  =   C * E;
	rot[0][1]  =  -C * F;
	rot[0][2]  =  D;

	rot[1][0]  = -BD * E + A * F;
	rot[1][1]  =  BD * F + A * E;
	rot[1][2]  =  -B * C;

	rot[2][0]  =  AD * E + B * F;
	rot[2][1]  = -AD * F + B * E;
	rot[2][2] =   A * C;

	return rot;
}



// convert rigid motion to opengl motion (column order)
template <class T1, class T2>
void to_opengl( const rigid_motion<T1> & rm, T2 * mat ) {
	mat[0] = rm.m_rotation[0][0];
	mat[1] = rm.m_rotation[1][0];
	mat[2] = rm.m_rotation[2][0];
	mat[3] = 0;
	mat[4] = rm.m_rotation[0][1];
	mat[5] = rm.m_rotation[1][1];
	mat[6] = rm.m_rotation[2][1];
	mat[7] = 0;
	mat[8] = rm.m_rotation[0][2];
	mat[9] = rm.m_rotation[1][2];
	mat[10] = rm.m_rotation[2][2];
	mat[11] = 0;
	mat[12] = rm.m_translation[0];
	mat[13] = rm.m_translation[1];
	mat[14] = rm.m_translation[2];
	mat[15] = 1;
}


// convert rigid motion to opengl motion (column order)
template <class T1, class T2>
void to_opengl( const math::matrix_fixed<T1,3,3> & rot, T2 * mat ) {
	mat[0] = rot[0][0];
	mat[1] = rot[1][0];
	mat[2] = rot[2][0];
	mat[3] = 0;
	mat[4] = rot[0][1];
	mat[5] = rot[1][1];
	mat[6] = rot[2][1];
	mat[7] = 0;
	mat[8] = rot[0][2];
	mat[9] = rot[1][2];
	mat[10] = rot[2][2];
	mat[11] = 0;
	mat[12] = 0;
	mat[13] = 0;
	mat[14] = 0;
	mat[15] = 1;
}


// convert rigid motion to opengl motion (column order)
template <class T1, class T2>
void to_opengl( const math::matrix_fixed<T1,4,4> & m, T2 * mat ) {
	mat[0] = m[0][0];
	mat[1] = m[1][0];
	mat[2] = m[2][0];
	mat[3] = m[3][0];
	mat[4] = m[0][1];
	mat[5] = m[1][1];
	mat[6] = m[2][1];
	mat[7] = m[3][1];
	mat[8] = m[0][2];
	mat[9] = m[1][2];
	mat[10] = m[2][2];
	mat[11] = m[3][2];
	mat[12] = m[0][3];
	mat[13] = m[1][3];
	mat[14] = m[2][3];
	mat[15] = m[3][3];
}

template<class T>
math::matrix_fixed<T,4,4> to_matrix_4x4( const rigid_motion<T> & rm) {
	math::matrix_fixed<T,4,4> mat;
	mat[0][0] = rm.m_rotation[0][0];
	mat[0][1] = rm.m_rotation[0][1];
	mat[0][2] = rm.m_rotation[0][2];
	mat[0][3] = rm.m_translation[0];
	mat[1][0] = rm.m_rotation[1][0];
	mat[1][1] = rm.m_rotation[1][1];
	mat[1][2] = rm.m_rotation[1][2];
	mat[1][3] = rm.m_translation[1];
	mat[2][0] = rm.m_rotation[2][0];
	mat[2][1] = rm.m_rotation[2][1];
	mat[2][2] = rm.m_rotation[2][2];
	mat[2][3] = rm.m_translation[2];
	mat[3][0] = 0;
	mat[3][1] = 0;
	mat[3][2] = 0;
	mat[3][3] = 1;

	return mat;
}


template<class T>
rigid_motion<float> to_float( const rigid_motion<T> & rm) {
	rigid_motion<float> rmd;
	for(unsigned int i = 0; i < 3; i++) {
		for(unsigned int j = 0; j < 3; j++) {
			rmd.m_rotation[i][j] = rm.m_rotation[i][j];
		}
		rmd.m_translation[i] = rm.m_translation[i];
	}
	return rmd;
}

template<class T>
rigid_motion<double> to_double( const rigid_motion<T> & rm) {
	rigid_motion<double> rmd;
	for(unsigned int i = 0; i < 3; i++) {
		for(unsigned int j = 0; j < 3; j++) {
			rmd.m_rotation[i][j] = rm.m_rotation[i][j];
		}
		rmd.m_translation[i] = rm.m_translation[i];
	}
	return rmd;
}


#endif /*RIGID_MOTION_HPP_*/
