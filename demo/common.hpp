#ifndef _COMMON_H_
#define _COMMON_H_

#include <assert.h>

// various constants
#ifndef FALSE
	#define FALSE					0
#endif

#ifndef TRUE
	#define TRUE					1
#endif

#ifndef NULL
	#define NULL					0
#endif

#define NOT_NULL					1

typedef unsigned long		DWORD;
typedef signed long			LONG;
typedef unsigned long		ULONG;
typedef unsigned short		WORD;
typedef signed short		SHORT;
typedef signed int			INT;
typedef unsigned int		UINT;
//typedef unsigned char		CHAR;
typedef unsigned char		BYTE;
typedef int					BOOL;
typedef float				FLOAT;
typedef double				DOUBLE;

#define KB(x) ((x)*1024)
#define MB(x) ((x)*1024*1024)

// min/max
#ifndef MIN
	#define MIN(a,b) 				(((a)<(b))?(a):(b))
#endif
#ifndef MAX
	#define MAX(a,b) 				(((a)>(b))?(a):(b))
#endif
#define MINMAX(a, b, c) (((b) < (a)) ? (a) : (((b) > (c)) ? (c) : (b)))

#define ABS(a)					(((a)<0)?(-a):(a))

// safe object deletion
#define DELETE_OBJECT(x)	if ((x)) { delete (x); (x) = NULL; }
#define DELETE_ARRAY(x)		if ((x)) { delete [] (x); (x) = NULL; }
#define DELETE_OBJECT_ARRAY(n,x)		if ((x)) { for (int i = 0; i < n; i++) { DELETE_OBJECT(((x)[i])); } delete [] (x); (x) = NULL; }

// math constants
#define PI				   3.141592653589793238512808959406186204433
#define PI_DOUBLE          6.283185307179586476925286766559
#define PI_HALF			   1.570796326794896619256404479703093102216

#define SQR(x) ((x)*(x))

#define BYTE2FLOAT(x)  (float(x)/255.0f)
#define FLOAT2BYTE(x)  ((BYTE) MINMAX(0, (x) * 255.0f, 255))

#define RAD_2_DEGREE(x)		((x)/PI*180.)
#define DEGREE_2_RAD(x)		((x)/180.*PI)

#endif //_COMMON_H_
