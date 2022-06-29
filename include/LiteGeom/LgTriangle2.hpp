/** @file
 Light 2D triangle class for use in MATIS: Lg::Triangle2(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgTriangle2_hpp
#define LgTriangle2_hpp

#include "LgSegment2.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 2D triangle class for use in MATIS: Lg::Triangle2(d, f)
	/** Defined by three points A, B and C */
	template <typename T> class TTriangle2 : std::vector< TPoint2<T> >
	{
		typedef std::vector< TPoint2<T> > mother;
	public:
		typedef T coord_type;

		/* ----- constructors ----- */
		/// Constructor from three (or less) points
		TTriangle2(TPoint2<T> A=TPoint2<T>(), TPoint2<T> B=TPoint2<T>(), TPoint2<T> C=TPoint2<T>()) {
			mother::push_back(A);
			mother::push_back(B);
			mother::push_back(C);
		}
		/// Constructor from an array of three points
		TTriangle2(T* data) {
			mother::push_back(TPoint2<T>(data));
			mother::push_back(TPoint2<T>(data+2));
			mother::push_back(TPoint2<T>(data+4));
		}
		/// Constructor from a 2D (3x2) array
		TTriangle2(T** data) {
			mother::push_back(TPoint2<T>(data[0]));
			mother::push_back(TPoint2<T>(data[1]));
			mother::push_back(TPoint2<T>(data[2]));
		}

		/* ----- access ----- */
		inline TPoint2<T> & A() {return mother::at(0);}
		inline TPoint2<T> A() const {return mother::at(0);}
		inline TPoint2<T> & B() {return mother::at(1);}
		inline TPoint2<T> B() const {return mother::at(1);}
		inline TPoint2<T> & C() {return mother::at(2);}
		inline TPoint2<T> C() const {return mother::at(2);}
		inline TPoint2<T> Point(size_t i) const {return mother::at(i%3);}
		inline TPoint2<T> & Point(size_t i) {return mother::at(i%3);}
		inline TPoint2<T> Segment(size_t i) const {return TSegment2<T>(Point(i), Point(i+1));}
		inline TPoint2<T> Vector(size_t i) const {return (Point(i+1) - Point(i));}
		inline TPoint2<T> operator[](size_t i) const {return Point(i);}
		inline TPoint2<T> & operator[](size_t i) {return Point(i);}

		/// type conversion
		template <typename U> TTriangle2<U> Convert() const
		{
			return TTriangle2<U>(
				A().Convert<U>(),
				B().Convert<U>(),
				C().Convert<U>());
		}

		/* ----- modifiers ----- */
		inline void SetA(TPoint2<T> A){A()=A;}
		inline void SetB(TPoint2<T> B){B()=B;}
		inline void SetC(TPoint2<T> C){C()=C;}

		/* ----- geometry ----- */
		inline T SignedArea() const {return T(0.5)*(B()-A())^(C()-A());}
		inline T Area() const {T a = SignedArea(); return (a > 0 ? a : -a);}
		inline T G() const {return (A()+B()+C())/T(3);}
		inline T Perimeter() const {return Vector(0).Norm() + Vector(1).Norm() + Vector(2).Norm();}

	};

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TTriangle2<T> & t1, const TTriangle2<U> & t2 )
	{
		return ( t1.A() == t2.A() && t1.B() == t2.B() && t1.C() == t2.C() );
	}
	template <typename T, typename U> bool operator != ( const TTriangle2<T> & t1, const TTriangle2<U> & t2 )
	{
		return ( t1.A() != t2.A() || t1.B() != t2.B() || t1.C() != t2.C() );
	}
	template <typename T, typename U> TTriangle2<T> operator + ( const TTriangle2<T> & t, const TPoint2<U> & vect )
	{
		return TTriangle2<T> (t.A() + vect, t.B() + vect, t.C() + vect);
	}
	template <typename T, typename U> void operator += ( const TTriangle2<T> & t, const TPoint2<U> & vect )
	{
		t = t + vect;
	}
	template <typename T, typename U> TTriangle2<T> operator - ( const TTriangle2<T> & t, const TPoint2<U> & vect )
	{
		return TTriangle2<T> (t.A() - vect, t.B() - vect, t.C() - vect);
	}
	template <typename T, typename U> void operator -= ( const TTriangle2<T> & t, const TPoint2<U> & vect )
	{
		t = t - vect;
	}

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TTriangle2<T> & t )
	{
		return ( os << "{" << t.A() << ";" << t.B() << ";" << t.C() << "}" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TTriangle2<T> & t )
	{
		char dummy;
		return ( is >> dummy >> t.A() >> dummy >> t.B() >> dummy >> t.C() >> dummy) ;
	}
	template <typename T, typename U> T * operator << ( T * data, const TTriangle2<U> & t )
	{
		return ( data << t.A() << t.B() << t.C());
	}
	template <typename T, typename U> T * operator >> ( T * data, TTriangle2<U> & t )
	{
		return ( data >> t.A() >> t.B() >> t.C() );
	}

	/* ----- typedefs ----- */
	typedef TTriangle2<double> Triangle2d;
	typedef TTriangle2<float> Triangle2f;
	typedef Triangle2d Triangle2;

}; // namespace Lg

#endif //LgTriangle2_hpp
