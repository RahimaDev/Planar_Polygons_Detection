/** @file
 Light 3D point/vector class for use in MATIS: Lg::Point3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgPoint3_hpp
#define LgPoint3_hpp

#include <vector>
#include <iostream>
#include <limits>
#include <math.h>

// useful macro for functions passing coords
#define LGCOORD3(P) P.x(), P.y(), P.z()

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 3D point/vector class for use in MATIS: Lg::Point3(d, f)
	template <typename T> class TPoint3
	{
	private:
		T mx, my, mz;

	public:
		typedef T coord_type;

		/* ----- constructors ----- */
		TPoint3 ( ) : mx(T(0)), my(T(0)), mz(T(0)) {}
		TPoint3 ( const T & x, const T & y, const T & z ) : mx(x), my(y), mz(z) {}
		template <typename U> TPoint3 ( const U * data  ) : mx(data[0]), my(data[1]), mz(data[2]) {}
		template <typename U> TPoint3 ( const TPoint3<U> & PU ) : mx(PU[0]), my(PU[1]), mz(PU[2]) {}

		/* ----- access ----- */
		inline T x() const {return mx;}
		inline T y() const {return my;}
		inline T z() const {return mz;}
		inline T & x() {return mx;}
		inline T & y() {return my;}
		inline T & z() {return mz;}
		inline void GetData(T * coords) const {coords[0]=mx; coords[1]=my; coords[2]=mz;}
		//inline void GetData(T& * coords) {coords[0]=mx; coords[1]=my; coords[2]=mz;}
		inline T Coord(size_t i) const {T coords[3]; GetData(coords); return coords[i%3];}
		inline T& Coord(size_t i) {i=i%3; return (i ? (i-1 ? mz : my) : mx);} //*(&mx+(i%3))
		inline T operator [](size_t i) const {return Coord(i);}
		inline T & operator [](size_t i) {return Coord(i);}

		/// type conversion
		template <typename U> TPoint3<U> Convert() const {return TPoint3<U>(U(mx), U(my), U(mz));}

		/* ----- modifiers ----- */
		inline void Setx(const T & x) { mx=T(x);}
		inline void Sety(const T & y) { my=T(y);}
		inline void Setz(const T & z) { mz=T(z);}
		inline void Setxyz(const T & x, const T & y, const T & z) { mx=T(x); my=T(y); mz=T(z);}
		inline void Translate(TPoint3<T> vect) {
			mx+=vect.x();
			my+=vect.y();
			mz+=vect.z();}
		inline T Normalize() {
			T norm = Norm();
			if(norm) {
				T norm_inv=1./Norm();
				mx*=norm_inv; my*=norm_inv; mz*=norm_inv;
				return norm_inv;
			}
			std::cout << "Warning: called Normalize() on a null Point3, setting to (1,0,0)" << std::endl;
			Setxyz(1,0,0);
			return 0;
		}

		/* ----- convenience ----- */
		inline T X() const {return mx;}
		inline T Y() const {return my;}
		inline T Z() const {return mz;}
		inline T & X() {return mx;}
		inline T & Y() {return my;}
		inline T & Z() {return mz;}
		inline void SetX(const T & x) { mx=T(x);}
		inline void SetY(const T & y) { my=T(y);}
		inline void SetZ(const T & z) { mz=T(z);}
		inline void SetXYZ(const T & x, const T & y, const T & z) { mx=T(x); my=T(y); mz=T(z);}

		/* ----- geometry ----- */
		inline T Norm2() const {return mx*mx + my*my + mz*mz;}
		inline T Norm() const {return sqrt(Norm2());}
		inline TPoint3<T> Normalized() const {
			T norm = Norm();
			if(norm) {
				T norm_inv = 1./Norm();
				return TPoint3<T> (norm_inv*mx, norm_inv*my, norm_inv*mz);
			}
			std::cout << "Warning: called Normalized() on a null Point3, returning (1,0,0)" << std::endl;
			return TPoint3<T>(1, 0, 0);
		}
	};

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return ( A.x() == T(B.x()) && A.y() == T(B.y()) && A.z() == T(B.z()) );
	}
	template <typename T, typename U> bool operator != ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return ( A.x() != T(B.x()) || A.y() != T(B.y()) || A.z() != T(B.z()) );
	}
	template <typename T, typename U> TPoint3<T> operator + ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return TPoint3<T> ( A.x() + T(B.x()), A.y() + T(B.y()), A.z() + T(B.z()) );
	}
	template <typename T, typename U> void operator += ( TPoint3<T> & A, const TPoint3<U> & B )
	{
		A = A + B;
	}
	template <typename T, typename U> TPoint3<T> operator - ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return TPoint3<T> ( A.x() - T(B.x()), A.y() - T(B.y()), A.z() - T(B.z()) );
	}
	template <typename T, typename U> void operator -= ( TPoint3<T> & A, const TPoint3<U> & B )
	{
		A = A - B;
	}
	template <typename T> TPoint3<T> operator - ( const TPoint3<T> & A )
	{
		return TPoint3<T> ( -A.x(), -A.y(), -A.z() );
	}
	// only exception to the return type=first arg type
	template <typename T, typename U> TPoint3<U> operator * ( const T & a, const TPoint3<U> & B )
	{
		U Ua=U(a);
		return TPoint3<U> ( Ua*B.x(), Ua*B.y(), Ua*B.z() );
	}
	template <typename T, typename U> TPoint3<T> operator * ( const TPoint3<T> & A, const U & b )
	{
		T Tb = T(b);
		return TPoint3<T> ( A.x()*Tb, A.y()*Tb, A.z()*Tb );
	}
	template <typename T, typename U> void operator *= ( TPoint3<T> & A, const U & b )
	{
		A = A * b;
	}
	template <typename T, typename U> TPoint3<T> operator / ( const TPoint3<T> & A, const U & b )
	{
		return A * ( T(1) / T(b) );
	}
	template <typename T, typename U> T operator * ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return T ( A.x()*T(B.x()) + A.y()*T(B.y()) + A.z()*T(B.z()) );
	}
	template <typename T, typename U> TPoint3<T> operator ^ ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return TPoint3<T> ( A.y()*T(B.z()) - A.z()*T(B.y()),
			A.z()*T(B.x()) - A.x()*T(B.z()),
			A.x()*T(B.y()) - A.y()*T(B.x()) );
	}

	/* ----- stream operators ----- */
	template <typename T, typename U> TPoint3<T> midpoint ( const TPoint3<T> & A, const TPoint3<U> & B )
	{
		return 0.5 * ( A + B );
	}

	template <typename T> std::ostream& operator << ( std::ostream &os, const TPoint3<T> & P )
	{
		return ( os << "(" << P.x() << "," << P.y() << "," << P.z() << ")" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TPoint3<T> & P )
	{
		char dummy;
		return ( is >> dummy >> P.x() >> dummy >> P.y() >> dummy  >> P.z() >> dummy ) ;
	}

	template <typename T, typename U> T * operator << ( T * data, const TPoint3<U> & P )
	{
		data[0]=T(P.x()); data[1]=T(P.y()); data[2]=T(P.z());
		return data+3;
	}

	template <typename T, typename U> T * operator >> ( T * data, TPoint3<U> & P )
	{
		P.x() = U(*data++);
		P.y() = U(*data++);
		P.z() = U(*data++);
		return data ;
	}


	/* ----- typedefs ----- */
	typedef TPoint3<double> Point3d;
	typedef TPoint3<float> Point3f;
	typedef Point3d Point3;

}; // namespace Lg

#endif //LgPoint3_hpp
