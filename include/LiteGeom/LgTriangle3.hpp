/** @file
 Light 3D triangle class for use in MATIS: Lg::Triangle3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgTriangle3_hpp
#define LgTriangle3_hpp

#include "LgPlane3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 3D triangle class for use in MATIS: Lg::Triangle3(d, f)
	/** Defined by three points A, B and C */
	template <class T> class TTriangle3
	{
	public:
		typedef T coord_type;
		/// constructor
		TTriangle3(
			TPoint3<T> A=TPoint3<T>(),
			TPoint3<T> B=TPoint3<T>(),
			TPoint3<T> C=TPoint3<T>() ): mA(A), mB(B), mC(C)
		{}

		/* ----- access ----- */
		inline TPoint3<T> & A() {return mA;}
		inline TPoint3<T> A() const {return mA;}
		inline TPoint3<T> & B() {return mB;}
		inline TPoint3<T> B() const {return mB;}
		inline TPoint3<T> & C() {return mC;}
		inline TPoint3<T> C() const {return mC;}
        inline TPoint3<T> AB() const {return mB-mA;}
        inline TPoint3<T> BC() const {return mC-mB;}
        inline TPoint3<T> CA() const {return mA-mC;}
		inline void GetData(TPoint3<T> * ABC) const {ABC[0]=mA; ABC[1]=mB; ABC[2]=mC;}
		//inline void GetData(TPoint3<T>& * ABC) {ABC[0]=mA; ABC[1]=mB; ABC[2]=mC;}
		inline TPoint3<T> Point(size_t i) const {TPoint3<T> ABC[3]; GetData(ABC); return ABC[i%3];}
		inline TPoint3<T> & Point(size_t i) {i=i%3; return (i ? (i-1 ? mC : mB) : mA);}
		inline TPoint3<T> Segment(size_t i) const {return Segment2(Point(i), Point(i+1));}
		inline TPoint3<T> operator[](size_t i) const {return Point(i);}
		inline TPoint3<T> & operator[](size_t i) {return Point(i);}

		/// type conversion
		template <typename U> TTriangle3<U> Convert() const
		{
			return TTriangle3<U>(
				A().Convert(),
				B().Convert(),
				C().Convert());
		}

		/* ----- modifiers ----- */
		inline void SetA(TPoint3<T> A){mA=A;}
		inline void SetB(TPoint3<T> B){mB=B;}
		inline void SetC(TPoint3<T> C){mC=C;}
		inline void SetABC(TPoint3<T> A, TPoint3<T> B, TPoint3<T> C){mA=A; mB=B; mC=C;}

		/* ----- shortcuts ----- */
		inline TPoint3<T> P(size_t i) const {return Point(i);}
		inline TPoint3<T> & P(size_t i) {return Point(i);}
		inline TPoint3<T> S(size_t i) const {return Segment(i);}

		/// geometric properties
		inline TPlane3<T> Plane() const {return TPlane3<T>(mA, mB, mC);}
		inline TPoint3<T> WeightedNormal() const {return (mB-mA)^(mC-mA);}
		inline TPoint3<T> Normal() const {return WeightedNormal().Normalized();}
		inline T Area() const {return T(0.5)*WeightedNormal().Norm();}
		inline T Area2() const {return T(0.25)*WeightedNormal().Norm2();}
		inline T G() const {return (mA+mB+mC)/T(3);}

	private:
		TPoint3<T> mA, mB, mC;
	};

	/* ----- algebraic operators ----- */
	template <class T> bool operator == ( const TTriangle3<T> & t1, const TTriangle3<T> & t2 )
	{
		return ( t1.A() == t2.A() && t1.B() == t2.B() && t1.C() == t2.C() );
	}
	template <class T> bool operator != ( const TTriangle3<T> & t1, const TTriangle3<T> & t2 )
	{
		return ( t1.A() != t2.A() || t1.B() != t2.B() || t1.C() != t2.C() );
	}
	template <class T> TTriangle3<T> operator + ( const TTriangle3<T> & t, const TPoint3<T> & vect )
	{
		return TTriangle3<T> (t.A() + vect, t.B() + vect, t.C() + vect);
	}
	template <class T> void operator += ( const TTriangle3<T> & t, const TPoint3<T> & vect )
	{
		t = t + vect;
	}
	template <class T> TTriangle3<T> operator - ( const TTriangle3<T> & t, const TPoint3<T> & vect )
	{
		return TTriangle3<T> (t.A() - vect, t.B() - vect, t.C() - vect);
	}
	template <class T> void operator -= ( const TTriangle3<T> & t, const TPoint3<T> & vect )
	{
		t = t - vect;
	}
	template <class T> std::ostream& operator << ( std::ostream &os, const TTriangle3<T> & t )
	{
                return ( os << "{" << t.A() << "; " << t.B() << "; " << t.C() << "}" ) ;
	}

	/* ----- typedefs ----- */
	typedef TTriangle3<double> Triangle3d;
	typedef TTriangle3<float> Triangle3f;
	typedef Triangle3d Triangle3;

}; // namespace Lg

#endif //LgTriangle3_hpp
