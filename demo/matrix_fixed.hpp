#ifndef MATRIX_FIXED_HPP_
#define MATRIX_FIXED_HPP_

#include "common.hpp"
#include "vector.hpp"
#include <string.h>
#include <iostream>

namespace math {
	template <class T, unsigned int R, unsigned int C>
	struct matrix_fixed;
}

typedef math::matrix_fixed<double,2,2>	math_matrix_2x2d;
typedef math::matrix_fixed<double,3,3>	math_matrix_3x3d;
typedef math::matrix_fixed<double,3,2>	math_matrix_3x2d;
typedef math::matrix_fixed<double,3,4>	math_matrix_3x4d;
typedef	math::matrix_fixed<double,4,3>	math_matrix_4x3d;
typedef	math::matrix_fixed<double,4,4>	math_matrix_4x4d;

typedef math::matrix_fixed<float,2,2>		math_matrix_2x2f;
typedef math::matrix_fixed<float,3,3>		math_matrix_3x3f;
typedef math::matrix_fixed<float,3,4>		math_matrix_3x4f;
typedef math::matrix_fixed<float,4,3> 	math_matrix_4x3f;
typedef math::matrix_fixed<float,4,4> 	math_matrix_4x4f;

namespace math {


/**
 * Simple matrix class for five_point algorithm. Only basic linalg operations are
 * defined. Compiler can perform many optimizations as it does not use library functions.
 *
 */
template <class T, unsigned int R, unsigned int C>
struct matrix_fixed {
	T v[R][C];

	const T * data_block() const { return &(v[0][0]); }
	T * data_block() { return &(v[0][0]); }

	T * operator[](unsigned int i) {
		return v[i];
	}

	const T * operator[](unsigned int i) const {
		return v[i];
	}

	T & operator()(unsigned int i, unsigned int j) {
		return v[i][j];
	}

	const T & operator()(unsigned int i, unsigned int j) const {
		return v[i][j];
	}

	bool operator==(const matrix_fixed & o) const {
		return ( memcmp(v, o.v,sizeof(T)*R*C) == 0 );
	}

	bool operator!=(const matrix_fixed & o) const {
		return ( memcmp(v, o.v,sizeof(T)*R*C) != 0 );
	}

	matrix_fixed & operator+=(const matrix_fixed & o) {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] += o.v[i][j];
			}
		}
		return *this;
	}

	matrix_fixed & operator-=(const matrix_fixed & o) {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] -= o.v[i][j];
			}
		}
		return *this;
	}

	template<class S>
	matrix_fixed & operator*=(const S & s) {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] *= s;
			}
		}
		return *this;
	}

	template<class S>
	matrix_fixed & operator/=(const S & s) {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] /= s;
			}
		}
		return *this;
	}

	void fill(T f) {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] = f;
			}
		}
	}

	void set_identity() {
		for(unsigned int i = 0; i < R; i++) {
			for(unsigned int j = 0; j < C; j++) {
				 v[i][j] = ( (i==j) ? T(1.0) : T(0.0) );
			}
		}
	}

	matrix_fixed<T,C,R> transpose() const {
		matrix_fixed<T,C,R> mat;
		for(unsigned i = 0; i < R; i++) {
			for(unsigned j = 0; j < C; j++) {
				mat[j][i] = v[i][j];
			}
		}
		return mat;
	}


	matrix_fixed<T,R,C> & set_row(int i, const vector_fixed<T,C> & vec)
	{
		for(unsigned int j = 0; j < C; j++) v[i][j] = vec[j];
		return *this;
	}


	vector_fixed<T,C> get_row(int i) const
	{
		vector_fixed<T,C> vec;
		for(unsigned int j = 0; j < C; j++) vec[j] = v[i][j];
		return vec;
	}

	matrix_fixed<T,R,C> & set_column(int i, const vector_fixed<T,R> & vec)
	{
		for(unsigned int j = 0; j < R; j++) v[j][i] = vec[j];
		return *this;
	}


	vector_fixed<T,R> get_column(int i) const
	{
		vector_fixed<T,R> vec;
		for(unsigned int j = 0; j < R; j++) vec[j] = v[j][i];
		return vec;
	}

	friend std::ostream & operator << (std::ostream &os, const matrix_fixed<T,R,C> & o){
		for(unsigned int i = 0; i < R; i++)
		{
			for(unsigned int j = 0; j < C; j++)
			{
				os << (o.v[i][j]) << " ";
			}
			os << std::endl;
		}
		return os;
	}


	friend std::istream & operator >> (std::istream &is, matrix_fixed<T,R,C> & o){
		for(unsigned int i = 0; i < R; i++)
		{
			for(unsigned int j = 0; j < C; j++)
			{
				is >> (o.v[i][j]);
			}
		}
		return is;
	}

	void copy_in( T * data ) {
		memcpy( v, data, sizeof(T)*C*R );
	}
	void copy_out( T * data ) const {
		memcpy( data, v, sizeof(T)*C*R );
	}


	T one_norm() const {
			double tot = 0;
			for(unsigned int i = 0; i < R; i++)
			{
				for(unsigned int j = 0; j < C; j++)
				{
					tot += fabs( v[i][j] );
				}
			}
			return 	tot;
	}

	T two_norm() const {
		double tot = 0;
		for(unsigned int i = 0; i < R; i++)
		{
			for(unsigned int j = 0; j < C; j++)
			{
				tot += v[i][j] * v[i][j];
			}
		}
		return 	sqrt( tot );
	}

	T inf_norm() const {
		double maxVal = 0;
		for(unsigned int i = 0; i < R; i++)
		{
			for(unsigned int j = 0; j < C; j++)
			{
				maxVal += std::max( fabs( v[i][j] ), maxVal );
			}
		}
		return maxVal;
	}

};


template<class T, unsigned int R, unsigned int C>
matrix_fixed<T,R,C> operator-(const matrix_fixed<T,R,C> & o) {
	matrix_fixed<T,R,C> mat;
	for(unsigned int i = 0; i < R; i++) {
		for(unsigned int j = 0; j < C; j++) {
			 mat.v[i][j] = -o.v[i][j];
		}
	}
	return mat;
}

template <class T, unsigned int R, unsigned int C>
vector_fixed<T,R> get_col(const matrix_fixed<T,R,C> & m, unsigned i) {
	vector_fixed<T,R> res;
	for(unsigned j = 0; j < R; j++) res.v[j] = m.v[j][i];
	return res;
}

template <class T, unsigned int C, unsigned int R>
void set_col(matrix_fixed<T,R,C> & m, unsigned i, const vector_fixed<T,R> & a) {
	for(unsigned j = 0; j < R; j++) m.v[j][i] = a.v[j];
}

template <class T, unsigned int R, unsigned int C>
vector_fixed<T,C> get_row(const matrix_fixed<T,R,C> & m, unsigned i) {
	vector_fixed<T,C> res;
	for(unsigned j = 0; j < C; j++) res.v[j] = m.v[i][j];
	return res;
}

template <class T, unsigned int R, unsigned int C>
void set_row(matrix_fixed<T,R,C> & m, unsigned i, const vector_fixed<T,C> & a) {
	for(unsigned j = 0; j < C; j++) m.v[i][j] = a.v[j];
}


template <class T, unsigned int R, unsigned int C>
inline vector_fixed<T,C> operator*(const matrix_fixed<T,R,C> & m, const vector_fixed<T,C> & a) {
	vector_fixed<T,R> res;

	for(unsigned int i = 0; i < R; i++) {
		res.v[i] = m.v[i][0] * a.v[0];
		for(unsigned int j = 1; j < C; j++) {
			 res.v[i] += m.v[i][j] * a.v[j];
		}
	}

	return res;
}


template <class T, unsigned int R, unsigned int C1, unsigned int C2>
matrix_fixed<T,R,C2> operator*(const matrix_fixed<T,R,C1> & m1, const matrix_fixed<T,C1,C2> & m2) {
	matrix_fixed<T,R,C2> res;

	for(unsigned int i = 0; i < R; i++) {
		for(unsigned int j = 0; j < C2; j++) {
			res.v[i][j] = m1.v[i][0] * m2.v[0][j];
			for(unsigned int k = 1; k < C1; k++) {
				res.v[i][j] += m1.v[i][k] * m2.v[k][j];
			}
		}
	}

	return res;
}

template <class T>
T det( const matrix_fixed<T,2,2> & m) {
	T d = m.v[0][0]*m.v[1][1]-m.v[0][1]*m.v[1][0];

	return d;
}


// returns the inverse of matrix
template< class T>
inline T det( const matrix_fixed<T,3,3> & mat)
{
	return mat.v[0][0]*mat.v[1][1]*mat.v[2][2]+mat.v[0][1]*mat.v[1][2]*mat.v[2][0]+mat.v[0][2]*mat.v[1][0]*mat.v[2][1]-mat.v[0][0]*mat.v[1][2]*mat.v[2][1]-mat.v[0][1]*mat.v[1][0]*mat.v[2][2]-mat.v[0][2]*mat.v[1][1]*mat.v[2][0];
}



template <class T, unsigned int R, unsigned int C>
inline matrix_fixed<T,R,C> operator*(T s, const matrix_fixed<T,R,C> & v) {
	matrix_fixed<T,R,C> res;
	for(unsigned i = 0; i < R; i++) {
		for(unsigned j = 0; j < C; j++) {
			res.v[i][j] = v.v[i][j] * s;
		}
	}
	return res;
}


template <class T, unsigned int R, unsigned int C>
inline matrix_fixed<T,R,C> operator*(const matrix_fixed<T,R,C> & v, T s) {
	matrix_fixed<T,R,C> res;
	for(unsigned i = 0; i < R; i++) {
		for(unsigned j = 0; j < C; j++) {
			res.v[i][j] = v.v[i][j] * s;
		}
	}
	return res;
}



template <class T, unsigned int R, unsigned int C>
inline matrix_fixed<T,R,C> operator+(const matrix_fixed<T,R,C> & v1, const matrix_fixed<T,R,C> & v2) {
	matrix_fixed<T,R,C> res;
	for(unsigned i = 0; i < R; i++) {
		for(unsigned j = 0; j < C; j++) {
			res.v[i][j] = v1.v[i][j] + v2.v[i][j];
		}
	}
	return res;
}

template <class T, unsigned int R, unsigned int C>
inline matrix_fixed<T,R,C> operator-(const matrix_fixed<T,R,C> & v1, const matrix_fixed<T,R,C> & v2) {
	matrix_fixed<T,R,C> res;
	for(unsigned i = 0; i < R; i++) {
		for(unsigned j = 0; j < C; j++) {
			res.v[i][j] = v1.v[i][j] - v2.v[i][j];
		}
	}
	return res;
}




// returns the inverse of matrix
template< class T>
inline matrix_fixed<T,2,2> inverse( const matrix_fixed<T,2,2> & mat)
{
	matrix_fixed<T,2,2> inv;

	T D = det(mat);
	T iD = 1.0/D;

	inv[0][0] = iD * mat[1][1];
	inv[0][1] = -iD * mat[0][1];
	inv[1][0] = -iD * mat[1][0];
	inv[1][1] = iD * mat[0][0];

	return inv;
}





// returns the inverse of matrix
template< class T>
inline matrix_fixed<T,3,3> inverse( const matrix_fixed<T,3,3> & mat, T det)
{
	matrix_fixed<T,3,3> inv;

	T idet=1./det;

	inv.v[0][0]=idet*(mat.v[1][1]*mat.v[2][2]-mat.v[2][1]*mat.v[1][2]);
	inv.v[0][1]=idet*(mat.v[0][2]*mat.v[2][1]-mat.v[2][2]*mat.v[0][1]);
	inv.v[0][2]=idet*(mat.v[0][1]*mat.v[1][2]-mat.v[1][1]*mat.v[0][2]);
	inv.v[1][0]=idet*(mat.v[1][2]*mat.v[2][0]-mat.v[2][2]*mat.v[1][0]);
	inv.v[1][1]=idet*(mat.v[0][0]*mat.v[2][2]-mat.v[2][0]*mat.v[0][2]);
	inv.v[1][2]=idet*(mat.v[0][2]*mat.v[1][0]-mat.v[1][2]*mat.v[0][0]);
	inv.v[2][0]=idet*(mat.v[1][0]*mat.v[2][1]-mat.v[2][0]*mat.v[1][1]);
	inv.v[2][1]=idet*(mat.v[0][1]*mat.v[2][0]-mat.v[2][1]*mat.v[0][0]);
	inv.v[2][2]=idet*(mat.v[0][0]*mat.v[1][1]-mat.v[1][0]*mat.v[0][1]);

	return inv;
}

// returns the inverse of matrix
template< class T>
inline matrix_fixed<T,3,3> inverse( const matrix_fixed<T,3,3> & mat)
{
	matrix_fixed<T,3,3> inv;

	T det= mat.v[0][0]*mat.v[1][1]*mat.v[2][2]+mat.v[0][1]*mat.v[1][2]*mat.v[2][0]+mat.v[0][2]*mat.v[1][0]*mat.v[2][1]-mat.v[0][0]*mat.v[1][2]*mat.v[2][1]-mat.v[0][1]*mat.v[1][0]*mat.v[2][2]-mat.v[0][2]*mat.v[1][1]*mat.v[2][0];

	T idet=1./det;

	inv.v[0][0]=idet*(mat.v[1][1]*mat.v[2][2]-mat.v[2][1]*mat.v[1][2]);
	inv.v[0][1]=idet*(mat.v[0][2]*mat.v[2][1]-mat.v[2][2]*mat.v[0][1]);
	inv.v[0][2]=idet*(mat.v[0][1]*mat.v[1][2]-mat.v[1][1]*mat.v[0][2]);
	inv.v[1][0]=idet*(mat.v[1][2]*mat.v[2][0]-mat.v[2][2]*mat.v[1][0]);
	inv.v[1][1]=idet*(mat.v[0][0]*mat.v[2][2]-mat.v[2][0]*mat.v[0][2]);
	inv.v[1][2]=idet*(mat.v[0][2]*mat.v[1][0]-mat.v[1][2]*mat.v[0][0]);
	inv.v[2][0]=idet*(mat.v[1][0]*mat.v[2][1]-mat.v[2][0]*mat.v[1][1]);
	inv.v[2][1]=idet*(mat.v[0][1]*mat.v[2][0]-mat.v[2][1]*mat.v[0][0]);
	inv.v[2][2]=idet*(mat.v[0][0]*mat.v[1][1]-mat.v[1][0]*mat.v[0][1]);

	return inv;
}

/*
template <class T, unsigned int C>
inline matrix_fixed<T,C,C> inverse( const matrix_fixed<T,C,C> & _mat ) {
	return from_vnl( vnl_svd<T>( to_vnl(_mat) ).inverse() );
}
*/



template<class T, class S, unsigned int ROWS, unsigned int COLS>
inline matrix_fixed<T,ROWS,COLS> to( const matrix_fixed<S,ROWS,COLS> & _mat ) {
	matrix_fixed<T,ROWS,COLS> m;
	for(unsigned int r=0; r < ROWS; r++)
		for(unsigned int c = 0; c < COLS; c++)
			m.v[r][c] = (T) _mat[r][c];
	return m;
}

template<class T, class S, unsigned int ROWS, unsigned int COLS>
inline matrix_fixed<T,ROWS,COLS> convert_to( const matrix_fixed<S,ROWS,COLS> & _mat ) {
	matrix_fixed<T,ROWS,COLS> m;
	for(unsigned int r=0; r < ROWS; r++)
		for(unsigned int c = 0; c < COLS; c++)
			m.v[r][c] = (T) _mat[r][c];
	return m;
}

template<class T, unsigned int ROWS, unsigned int COLS>
inline matrix_fixed<float,ROWS,COLS> to_float( const matrix_fixed<T,ROWS,COLS> & _mat ) {
	matrix_fixed<float,ROWS,COLS> m;
	for(unsigned int r=0; r < ROWS; r++)
		for(unsigned int c = 0; c < COLS; c++)
			m.v[r][c] = (float) _mat[r][c];
	return m;
}

template<class T, unsigned int ROWS, unsigned int COLS>
inline matrix_fixed<double,ROWS,COLS> to_double( const matrix_fixed<T,ROWS,COLS> & _mat ) {
	matrix_fixed<double,ROWS,COLS> m;
	for(unsigned int r=0; r < ROWS; r++)
		for(unsigned int c = 0; c < COLS; c++)
			m[r][c] = _mat[r][c];
	return m;
}


/**
 * generate outer product matrix
 */
template<class T, unsigned int C>
matrix_fixed<T,C,C> outer_product( const vector_fixed<T,C> & a, const vector_fixed<T,C> & b ) {
	matrix_fixed<T,C,C> mat;
	for(int i = 0; i < int(C); i++) {
		for(int j = 0; j < int(C); j++) {
			mat[i][j] = a[i]*b[j];
		}
	}
	return mat;
}

/**
 * generate cross product matrix from vector
 */
template<class T>
matrix_fixed<T,3,3> cross_product_matrix( const vector_fixed<T,3> & v )
{
	matrix_fixed<T,3,3> mat;
	mat[0][0] = 0;
	mat[0][1] = -v[2];
	mat[0][2] = v[1];
	mat[1][0] = v[2];
	mat[1][1] = 0;
	mat[1][2] = -v[0];
	mat[2][0] = -v[1];
	mat[2][1] = v[0];
	mat[2][2] = 0;
	return mat;
}

} // namespace math

#endif /* MATRIX_FIXED_HPP_*/
