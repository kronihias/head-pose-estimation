#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include "common.hpp"

#include "vector.hpp"
#include "matrix_fixed.hpp"
#include <iostream>

namespace math {


/**
 * Simple matrix class for five_point algorithm. Only basic linalg operations are
 * defined. Compiler can perform many optimizations as it does not use library functions.
 */
template <class T>
struct matrix {
	int m_rows;
	int m_cols;
	T * v;

	matrix() { m_rows = 0; m_cols = 0; v = new T[m_rows*m_cols]; }
	matrix(int rows, int cols) { m_rows = rows; m_cols = cols; v = new T[m_rows*m_cols]; }
	matrix(const matrix & other) {
		m_rows = other.m_rows;
		m_cols = other.m_cols;
		v = new T[m_rows*m_cols];
		memcpy( v, other.v, sizeof(T)*m_rows*m_cols);
	}
	~matrix() { delete [] v; }

	int cols() const { return m_cols; }
	int rows() const { return m_rows; }

	const T * data_block() const { return v; }
	T * data_block() { return v; }

	T * operator[](unsigned int i) {
		return &(v[m_cols*i]);
	}

	const T * operator[](unsigned int i) const {
		return &(v[m_cols*i]);
	}

	matrix & operator=( const matrix & other ) {
		if( m_rows != other.m_rows || m_cols != other.m_cols ) {
			delete [] v;
			m_rows = other.m_rows;
			m_cols = other.m_cols;
			v = new T[m_rows*m_cols];
		}
		memcpy( v, other.v, sizeof(T)*m_rows*m_cols );
		return *this;
	}



	T & operator()(unsigned int i, unsigned int j) {
		return v[m_cols*i+j];
	}

	const T & operator()(unsigned int i, unsigned int j) const {
		return v[m_cols*i+j];
	}


	matrix<T> & set_row(int i, const vector<T> & vec)
	{
		assert(cols() == vec.size());
		assert( i < rows() );
		for( int j = 0; j < cols(); j++) v[i*cols()+j] = vec[j];
		return *this;
	}


	vector<T> get_row(int i) const
	{
		assert( i < rows() );
		vector<T> vec(cols());
		for(int j = 0; j < cols(); j++) vec[j] = v[i*cols()+j];
		return vec;
	}

	matrix<T> & set_column(int i, const vector<T> & vec)
	{
		assert( rows() == vec.size() );
		assert( i < cols() );
		for(unsigned int j = 0; j < rows(); j++) v[j*cols()+i] = vec[j];
		return *this;
	}


	vector<T> get_column(int i) const
	{
		assert( i < cols() );
		vector<T> vec( rows() );
		for(int j = 0; j < rows(); j++) vec[j] = v[j*cols()+i];
		return vec;
	}

	matrix<T> transpose() const {
		matrix<T> out( cols() , rows() );
		for(int i = 0; i < rows(); i++)
		{
			for(int j = 0; j < cols(); j++)
			{
				out.v[j*rows()+i] = v[i*cols()+j];
			}
		}
		return out;
	}

	friend std::ostream & operator << (std::ostream &os, const matrix<T> & o){
		for(unsigned int i = 0; i < o.m_rows; i++)
		{
			for(unsigned int j = 0; j < o.m_cols; j++)
			{
				os << (o.v[i*o.m_cols+j]) << " ";
			}
			os << std::endl;
		}
		return os;
	}


	friend std::istream & operator >> (std::istream &is, matrix<T> & o){
		for(unsigned int i = 0; i < o.m_rows; i++)
		{
			for(unsigned int j = 0; j < o.m_cols; j++)
			{
				is >> (o.v[i*o.m_cols+j]);
			}
		}
		return is;
	}


	T one_norm() const {
			double tot = 0;
			for(unsigned int i = 0; i < m_rows; i++)
			{
				for(unsigned int j = 0; j < m_cols; j++)
				{
					tot += fabs( v[i*cols()+j] );
				}
			}
			return 	tot;
	}

	T two_norm() const {
		double tot = 0;
		for(unsigned int i = 0; i < m_rows; i++)
		{
			for(unsigned int j = 0; j < m_cols; j++)
			{
				tot += v[i*cols()+j] * v[i*cols()+j];
			}
		}
		return 	sqrt( tot );
	}

	T inf_norm() const {
		double maxVal = 0;
		for(unsigned int i = 0; i < m_rows; i++)
		{
			for(unsigned int j = 0; j < m_cols; j++)
			{
				maxVal += std::max( fabs( v[i*cols()+j] ), maxVal );
			}
		}
		return maxVal;
	}

};


template <class T>
inline vector<T> operator*(const matrix<T> & m, const vector<T> & a) {
	vector<T> res(m.rows());
	assert( m.cols() == a.size() );

	for(int i = 0; i < m.rows(); i++) {
		res[i] = m[i][0] * a[0];
		for(int j = 1; j < m.cols(); j++) {
			 res[i] += m[i][j] * a[j];
		}
	}

	return res;
}



template <class T, unsigned int R, unsigned int C>
matrix<T> operator*(const matrix_fixed<T,R,C> & m1, const matrix<T> & m2) {
	assert( C == m2.rows() );
	matrix<T> res(R,m2.cols());

	for(unsigned int i = 0; i < R; i++) {
		for( int j = 0; j < m2.cols(); j++) {
			res[i][j] = m1[i][0] * m2[0][j];
			for(unsigned int k = 1; k < C; k++) {
				res[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}

	return res;
}


template <class T>
matrix<T> operator*(const matrix<T> & m1, const matrix<T> & m2) {
	assert( m1.cols() == m2.rows() );
	matrix<T> res(m1.rows(),m2.cols());

	for(int i = 0; i < m1.rows(); i++) {
		for(int j = 0; j < m2.cols(); j++) {
			res[i][j] = m1[i][0] * m2[0][j];
			for(int k = 1; k < m1.cols(); k++) {
				res[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}

	return res;
}


template<class T, class S>
inline matrix<T> convert_to( const matrix<S> & _mat ) {
	matrix<T> m(_mat.rows(),_mat.cols());
	for( int r=0; r < _mat.rows(); r++)
		for( int c = 0; c < _mat.cols(); c++)
			m[r][c] = (T) _mat[r][c];
	return m;
}


} // namespace math

#endif /* MATRIX_HPP_*/
