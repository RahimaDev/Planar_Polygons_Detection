/** @file
 Light 2D points/vectors class for use in MATIS: LgPoint2(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgPoint2_hpp
#define LgPoint2_hpp

#include <vector>
#include <iostream>
#include <limits>

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 2D points/vectors class for use in MATIS: LgPoint2(d, f)
	template <typename T> class TPoint2
	{
	private:
		T mx, my;

	public:
		typedef T coord_type;

		/* ----- constructors ----- */
		TPoint2 ( ) : mx(T(0)), my(T(0)) {}
		TPoint2 ( const T & x, const T & y ) : mx(x), my(y) {}
		template <typename U> TPoint2 ( const U * data  ) : mx(data[0]), my(data[1]) {}
		template <typename U> TPoint2 ( const TPoint2<U> & PU ) : mx(PU[0]), my(PU[1]) {}

		/* ----- access ----- */
		inline T x() const {return mx;}
		inline T y() const {return my;}
		inline T & x() {return mx;}
		inline T & y() {return my;}
		inline void GetData(T * coords) const {coords[0]=mx; coords[1]=my;}
		// inline void GetData(T& * coords) {coords[0]=mx; coords[1]=my;}
		inline T Coord(size_t i) const {T coords[2]; GetData(coords); return coords[i%2];}
		inline T& Coord(size_t i) {return (i%2 ? my : mx);} //*(&mx+(i%2));
		inline T operator [](size_t i) const {return Coord(i);}
		inline T & operator [](size_t i) {return Coord(i);}

		/// type conversion
		template <typename U> TPoint2<U> Convert() const {return TPoint2<U>(U(mx), U(my));}

		/* ----- modifiers ----- */
		inline void Setx(const T & x) { mx=T(x);}
		inline void Sety(const T & y) { my=T(y);}
		inline void Setxy(const T & x, const T & y) { mx=T(x); my=T(y);}
		inline T Normalize() {
			T norm = Norm();
			if(norm) {
				T norm_inv=1./Norm();
				mx*=norm_inv; my*=norm_inv;
				return norm_inv;
			}
			std::cout << "Warning: called Normalize() on a null Point2, setting to (1,0)" << std::endl;
			Setxy(1,0);
			return 0;
		}

		/* ----- convenience ----- */
		inline T X() const {return mx;}
		inline T Y() const {return my;}
		inline T & X() {return mx;}
		inline T & Y() {return my;}
		inline void SetX(const T & x) { mx=T(x);}
		inline void SetY(const T & y) { my=T(y);}
		inline void SetXY(const T & x, const T & y) { mx=T(x); my=T(y);}

		/* ----- geometry ----- */
		inline T Norm2() const {return mx*mx + my*my;}
		inline T Norm() const {return sqrt(Norm2());}
		inline TPoint2<T> Normalized() const {
			T norm = Norm();
			if(norm) {
				T norm_inv = 1./Norm();
				return TPoint2<T> (norm_inv*mx, norm_inv*my);
			}
			std::cout << "Warning: called Normalized() on a null Point2, returning (1,0)" << std::endl;
			return TPoint2<T>(1, 0);
		}

	};

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return ( A.x() == T(B.x()) && A.y() == T(B.y()) );
	}
	template <typename T, typename U> bool operator == ( const TPoint2<T> & A, const U & b )
	{
		return ( A.x() == T(b) && A.y() == T(b) );
	}
	template <typename T, typename U> bool operator != ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return ( A.x() != T(B.x()) || A.y() != T(B.y()) );
	}
	template <typename T, typename U> bool operator != ( const TPoint2<T> & A, const U & b )
	{
		return ( A.x() != T(b) || A.y() != T(b) );
	}
	template <typename T, typename U> TPoint2<T> operator + ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return TPoint2<T> ( A.x() + T(B.x()), A.y() + T(B.y()) );
	}
	template <typename T, typename U> void operator += ( TPoint2<T> & A, const TPoint2<U> & B )
	{
		A = A + B;
	}
	template <typename T, typename U> TPoint2<T> operator - ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return TPoint2<T> ( A.x() - T(B.x()), A.y() - T(B.y()) );
	}
	template <typename T, typename U> void operator -= ( TPoint2<T> & A, const TPoint2<U> & B )
	{
		A = A - B;
	}
	template <typename T> TPoint2<T> operator - ( const TPoint2<T> & A )
	{
		return TPoint2<T> ( -A.x(), -A.y() );
	}
	// only exception to the return type=first arg type
	template <typename T, typename U> TPoint2<U> operator * ( const T & a, const TPoint2<U> & B )
	{
		return TPoint2<U> ( U(a)*B.x(), U(a)*B.y() );
	}
	template <typename T, typename U> TPoint2<T> operator * ( const TPoint2<T> & A, const U & b )
	{
		return TPoint2<T> ( A.x()*T(b), A.y()*T(b) );
	}
	template <typename T, typename U> void operator *= ( TPoint2<T> & A, const U & b )
	{
		A = A * T(b);
	}
	template <typename T, typename U> TPoint2<T> operator / ( const TPoint2<T> & A, const U & b )
	{
		return A * ( T(1) / T(b) );
	}
	template <typename T, typename U> void operator /= ( TPoint2<T> & A, const U & b )
	{
		A = A / b;
	}
	template <typename T, typename U> T operator * ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return T ( A.x()*T(B.x()) + A.y()*T(B.y()) );
	}
	template <typename T, typename U> T operator ^ ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return T ( A.x()*T(B.y()) - A.y()*T(B.x()) );
	}
	template <typename T, typename U> TPoint2<T> midpoint ( const TPoint2<T> & A, const TPoint2<U> & B )
	{
		return 0.5 * ( A + B );
	}

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TPoint2<T> & P )
	{
		return ( os << "(" << P.x() << "," << P.y() << ")" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TPoint2<T> & P )
	{
		char dummy;
		return ( is >> dummy >> P.x() >> dummy >> P.y() >> dummy ) ;
	}
	template <typename T, typename U> T * operator << ( T * data, const TPoint2<U> & P )
	{
		data[0] = T(P.x()); data[1] = T(P.y());
		return data+2 ;
	}
	template <typename T, typename U> T * operator >> ( T * data, TPoint2<U> & P )
	{
		P.x() = U(*data++);
		P.y() = U(*data++);
		return data ;
	}

	/* ----- typedefs ----- */
	typedef TPoint2<double> Point2d;
	typedef TPoint2<float> Point2f;
	typedef Point2d Point2;

}; // namespace Lg

// useful macro for functions passing coords
#define LGCOORD2(P) P.x(), P.y()

#endif //LgPoint2_hpp
