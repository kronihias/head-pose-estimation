#ifndef VECTOR_HPP_
#define VECTOR_HPP_

#include "common.hpp"

#include <iostream>
#include "vector_fixed.hpp"
#include <math.h>


namespace math {

template <class T>
struct vector{
	int m_size;
	T * v;

	const T * data_block() const { return v; }
	T * data_block() { return v; }

	vector() {
		m_size = 0;
		v = new T[m_size];
	}

	vector(int size) {
		m_size = size;
		v = new T[m_size];
	}

	vector(int size, T value) {
		m_size = size;
		v = new T[m_size];
		for(int i = 0; i < m_size; i++) v[i] = value;
	}

	vector(const vector & other) {
		m_size = other.m_size;
		v = new T[m_size];
		memcpy( v, other.v, sizeof(T)*m_size );
	}

	~vector() {
		delete [] v;
	}

	int size() const { return m_size; }


	vector & operator=( const vector & other ) {
		if( m_size != other.m_size ) {
			delete [] v;
			m_size = other.m_size;
			v = new T[m_size];
		}
		memcpy( v, other.v, sizeof(T)*m_size );
		return *this;
	}

	T & operator[](unsigned int i) {
		return v[i];
	}

	const T & operator[](unsigned int i) const {
		return v[i];
	}

	T & operator()(unsigned int i) {
		return v[i];
	}

	const T & operator()(unsigned int i) const {
		return v[i];
	}

	vector & operator*=( T s ) {
		for(int i = 0; i < size(); i++) v[i] *= s;
		return *this;
	}

	vector & operator/=( T s ) {
		for(int i = 0; i < size(); i++) v[i] /= s;
		return *this;
	}

	vector operator+(const vector & o) const {
		assert( o.size() == size() );
		vector out(size());
		for(int i = 0; i < size(); i++) out.v[i] = v[i] + o.v[i];
		return out;
	}

	vector operator-(const vector & o) const {
		assert( o.size() == size() );
		vector out(size());
		for(int i = 0; i < size(); i++) out.v[i] = v[i] - o.v[i];
		return out;
	}

	friend std::ostream & operator << (std::ostream &os, const vector<T> & o){
			for(unsigned int j = 0; j < o.m_size; j++)
			{
				os << (o.v[j]) << " ";
			}
			os << std::endl;
			return os;
	}


	friend std::istream & operator >> (std::istream &is, vector<T> & o){
		for(unsigned int j = 0; j < o.m_size; j++)
		{
			is >> (o.v[j]);
		}
		return is;
	}

	T one_norm() const {
			double tot = 0;
			for(int i = 0; i < size(); i++) tot += fabs( v[i] );
			return 	tot;
	}

	T two_norm() const {
		double tot = 0;
		for(int i = 0; i < size(); i++) tot += v[i] * v[i];
		return 	::sqrt( tot );
	}

	T inf_norm() const {
		double maxVal = 0;
		for(int i = 0; i < size(); i++) maxVal += std::max( fabs( v[i] ), maxVal );
		return maxVal;
	}

};



template <class T>
T dot_product(const vector<T> & a, const vector<T> & b)  {
	assert(a.size()==b.size());
	T sum = a[0] * b[0];
	for(int i = 1; i < a.size(); i++) sum += a[i] * b[i];
	return sum;
}


template <class T, unsigned int C>
T dot_product(const vector_fixed<T,C> & a, const vector<T> & b)  {
	assert(b.size() == C);
	T sum = a[0] * b[0];
	for(unsigned i = 1; i < C; i++) sum += a[i] * b[i];
	return sum;
}



template<class T, class S>
inline vector<T> convert_to( const vector<S> & _vec ) {
	vector<T> m(_vec.size());
	for( int r=0; r <_vec.size(); r++)
		m[r] = (T) _vec[r];
	return m;
}


}


#endif /*VECTOR_HPP_*/
