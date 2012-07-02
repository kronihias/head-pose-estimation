#ifndef VECTOR_FIXED_HPP_
#define VECTOR_FIXED_HPP_

#include "common.hpp"

#include <iostream>

namespace math {
	template <class T, unsigned int C>
	struct vector_fixed;
}

typedef math::vector_fixed<double,1> 		Vector1;
typedef math::vector_fixed<double,2> 		Vector2;
typedef math::vector_fixed<double,3> 		Vector3;

typedef math::vector_fixed<float,1>		Vector1f;
typedef math::vector_fixed<float,2>		Vector2f;
typedef math::vector_fixed<float,3>		Vector3f;

typedef math::vector_fixed<double,1>		Vector1d;
typedef math::vector_fixed<double,2>		Vector2d;
typedef math::vector_fixed<double,3>		Vector3d;

typedef math::vector_fixed<BYTE,1>		Vector1b;
typedef math::vector_fixed<BYTE,2>		Vector2b;
typedef math::vector_fixed<BYTE,3>		Vector3b;

typedef math::vector_fixed<int,1>		Vector1i;
typedef math::vector_fixed<int,2>		Vector2i;
typedef math::vector_fixed<int,3>		Vector3i;

typedef math::vector_fixed<double,1> math_vector_1d;
typedef math::vector_fixed<double,2> math_vector_2d;
typedef math::vector_fixed<double,3> math_vector_3d;

typedef math::vector_fixed<float,1> math_vector_1f;
typedef math::vector_fixed<float,2> math_vector_2f;
typedef math::vector_fixed<float,3> math_vector_3f;

typedef math::vector_fixed<int,1> math_vector_1i;
typedef math::vector_fixed<int,2> math_vector_2i;
typedef math::vector_fixed<int,3> math_vector_3i;

typedef math::vector_fixed<BYTE,1> math_vector_1b;
typedef math::vector_fixed<BYTE,2> math_vector_2b;
typedef math::vector_fixed<BYTE,3> math_vector_3b;

namespace math {


template <class T, unsigned int C>
struct vector_fixed{
	T v[C];

	const T * data_block() const { return v; }
	T * data_block() { return v; }

	vector_fixed() {}

	vector_fixed(T v0, T v1) {
		v[0]=v0;
		v[1]=v1;
	}

	vector_fixed(T v0, T v1, T v2) {
		v[0]=v0;
		v[1]=v1;
		v[2]=v2;
	}

	vector_fixed(T v0, T v1, T v2, T v3) {
			v[0]=v0;
			v[1]=v1;
			v[2]=v2;
			v[3]=v3;
	}

	vector_fixed(T * data) {
			copy_in(data);
	}

	T & operator[](unsigned int i) {
		return v[i];
	}

	const T & operator[](unsigned int i) const {
		return v[i];
	}

	bool operator==(const vector_fixed & o) const {
		return ( memcmp(v, o.v,sizeof(T)*C) == 0 );
	}

	bool operator!=(const vector_fixed & o) const {
		return ( memcmp(v, o.v,sizeof(T)*C) != 0 );
	}

	vector_fixed & operator+=(const vector_fixed & o) {
		for(unsigned i = 0; i < C; i++) v[i] += o.v[i];
		return *this;
	}

	vector_fixed & operator-=(const vector_fixed & o) {
		for(unsigned i = 0; i < C; i++) v[i] -= o.v[i];
		return *this;
	}

	vector_fixed & operator*=(T o) {
		for(unsigned i = 0; i < C; i++) v[i] *= o;
		return *this;
	}

	vector_fixed & operator/=(T o) {
		for(unsigned i = 0; i < C; i++) v[i] /= o;
		return *this;
	}

	vector_fixed operator*(T o) const {
		vector_fixed vec;
		for(unsigned i = 0; i < C; i++) vec.v[i] = v[i] * o;
		return vec;
	}

	vector_fixed operator/(T o) {
		vector_fixed vec;
		for(unsigned i = 0; i < C; i++) vec.v[i] = v[i] / o;
		return vec;
	}

	vector_fixed & normalize() {
		return (*this) /= length(*this);
	}

	void fill(T f) {
		for(unsigned i = 0; i< C; i++) v[i] = f;
	}

	T magnitude() const {
		return length(*this);
	}

	T squared_magnitude() const {
		return length2(*this);
	}

	T two_norm() const {
		return sqrt(dot_product(*this));
	}

	T dot_product(const vector_fixed & o) const {
		T sum = v[0] * o.v[0];
		for(unsigned i = 1; i < C; i++) sum += v[i] * o.v[i];
		return sum;
	}

	vector_fixed cross_product(const vector_fixed & o) const {
		assert(C==3);
		vector_fixed<T,3> r;
		r.v[0] = v[1]*o.v[2]-v[2]*o.v[1];
		r.v[1] = v[2]*o.v[0]-v[0]*o.v[2];
		r.v[2] = v[0]*o.v[1]-v[1]*o.v[0];
		return r;
	}

	friend std::ostream & operator << (std::ostream &os, const vector_fixed<T,C> & o){
			for(unsigned int j = 0; j < C; j++)
			{
				os << (o.v[j]) << " ";
			}
			os << std::endl;
			return os;
	}



	friend std::istream & operator >> (std::istream &is, vector_fixed<T,C> & o){
		for(unsigned int j = 0; j < C; j++)
		{
			is >> (o.v[j]);
		}
		return is;
	}

	void copy_in( T * data ) {
		memcpy( v, data, sizeof(T)*C );
	}
	void copy_out( T * data ) const {
		memcpy( data, v, sizeof(T)*C );
	}
};



template <class T, unsigned int C>
inline vector_fixed<T,C> operator-(const vector_fixed<T,C> & v1) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = -v1.v[i];
	return res;
}

template <class T, unsigned int C>
inline vector_fixed<T,C> operator+(const vector_fixed<T,C> & v1, const vector_fixed<T,C> & v2) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = v1.v[i] + v2.v[i];
	return res;
}

template <class T, unsigned int C>
inline vector_fixed<T,C> operator-(const vector_fixed<T,C> & v1, const vector_fixed<T,C> & v2) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = v1.v[i] - v2.v[i];
	return res;
}


template <class T, unsigned int C>
inline vector_fixed<T,C> operator/(const vector_fixed<T,C> & v, T s) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = v.v[i] / s;
	return res;
}

template <class T, unsigned int C>
inline vector_fixed<T,C> operator*(const vector_fixed<T,C> & v, T s) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = v.v[i] * s;
	return res;
}

template <class T, unsigned int C>
inline vector_fixed<T,C> operator*(T s, const vector_fixed<T,C> & v) {
	vector_fixed<T,C> res;
	for(unsigned i = 0; i < C; i++) res.v[i] = v.v[i] * s;
	return res;
}


template <class T>
inline vector_fixed<T,3> cross_product(const vector_fixed<T,3> & a, const vector_fixed<T,3> & b)  {
	vector_fixed<T,3> r;
	r.v[0] = a.v[1]*b.v[2]-a.v[2]*b.v[1];
	r.v[1] = a.v[2]*b.v[0]-a.v[0]*b.v[2];
	r.v[2] = a.v[0]*b.v[1]-a.v[1]*b.v[0];
	return r;
}

template <class T, unsigned int C>
inline T dot(const vector_fixed<T,C> & a, const vector_fixed<T,C> & b)  {
	T sum = a.v[0] * b.v[0];
	for(unsigned i = 1; i < C; i++) sum += a.v[i] * b.v[i];
	return sum;
}

template <class T, unsigned int C>
inline T dot_product(const vector_fixed<T,C> & a, const vector_fixed<T,C> & b)  {
	T sum = a.v[0] * b.v[0];
	for(unsigned i = 1; i < C; i++) sum += a.v[i] * b.v[i];
	return sum;
}

template <class T, unsigned int C>
T length2(const vector_fixed<T,C> & a) {
	return dot(a,a);
}

template <class T, unsigned int C>
T length(const vector_fixed<T,C> & a) {
	return (T) sqrt(length2(a));
}

template<class T, unsigned int C, class S>
inline vector_fixed<T,C> to( const vector_fixed<S,C> & _vec ) {
	vector_fixed<T,C> v;
	for(unsigned int r=0; r < C; r++)
			v.v[r] = _vec.v[r];
	return v;
}

template<class T, unsigned int C>
inline vector_fixed<float,C> to_float( const vector_fixed<T,C> & _vec ) {
	return to<float>(_vec);
}

template<class T, unsigned int C>
inline vector_fixed<double,C> to_double( const vector_fixed<T,C> & _vec ) {
	return to<double>(_vec);
}


}


#endif /*VECTOR_FIXED_HPP_*/
