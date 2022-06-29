/** @file
 Light 2D line class for use in MATIS: Lg::Line2(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgLine2_hpp
#define LgLine2_hpp

#include "LgPoint2.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 2D line class for use in MATIS: Lg::Line2(d, f)
	/** Defined by a Point (first, A) and a Vector (second, V) */
	template <typename T> class TLine2 : public std::pair< TPoint2<T>, TPoint2<T> >
	{
		typedef std::pair< TPoint2<T>, TPoint2<T> > mother;
	public:
		typedef T coord_type;
		/* ----- constructors ----- */
		TLine2(const T & ax, const T & ay, const T & vx, const T & vy):
		mother(TPoint2<T>(ax,ay), TPoint2<T>(vx,vy)){}
		TLine2(TPoint2<T> A = TPoint2<T>(), TPoint2<T> V = TPoint2<T>()): mother(A, V){}
		TLine2(T* data): mother(TPoint2<T>(data), TPoint2<T>(data+2)){}
		TLine2(T** data): mother(TPoint2<T>(data[0]), TPoint2<T>(data[1])){}

		/* ----- access ----- */
		inline TPoint2<T> & A() {return mother::first;}
		inline TPoint2<T> A() const {return mother::first;}
		inline TPoint2<T> Point() const {return mother::first;}
		inline TPoint2<T> & Point() {return mother::first;}
                inline TPoint2<T> Point(T x) const {return mother::first + x*mother::second;}
		inline TPoint2<T> B() const {return mother::first+mother::second;}
		inline TPoint2<T> & V() {return mother::second;}
		inline TPoint2<T> V() const {return mother::second;}
		inline TPoint2<T> & Vector() {return mother::second;}
		inline TPoint2<T> Vector() const {return mother::second;}

		/// type conversion
		template <typename U> TLine2<U> Convert() const
		{
			return TLine2<U>(A().Convert(), V().Convert());
		}

		/* ----- modifiers ----- */
		inline void SetPoint(TPoint2<T> pt){mother::first=pt;}
		inline void SetVector(TPoint2<T> vect){mother::second=vect;}
		inline void SetPointAndVector(TPoint2<T> pt, TPoint2<T> vect){mother::first=pt; mother::second=vect;}
		inline void SetA(TPoint2<T> A){mother::first=A;}
		inline void SetV(TPoint2<T> V){mother::second=V;}
		inline void SetAV(TPoint2<T> A, TPoint2<T> V){mother::first=A; mother::second=V;}

	};

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TLine2<T> & l1, const TLine2<U> & l2 )
	{
		return ( l1.A() == l2.A() && l1.V() == l2.V() );
	}
	template <typename T, typename U> bool operator != ( const TLine2<T> & l1, const TLine2<U> & l2 )
	{
		return ( l1.A() != l2.A() || l1.V() != l2.V() );
	}
	template <typename T, typename U> TLine2<T> operator + ( const TLine2<T> & l, const TPoint2<U> & vect )
	{
		return TLine2<T> (l.A() + vect, l.V());
	}
	template <typename T, typename U> void operator += ( const TLine2<T> & l, const TPoint2<U> & vect )
	{
		l = l + vect;
	}
	template <typename T, typename U> TLine2<T> operator - ( const TLine2<T> & l, const TPoint2<U> & vect )
	{
		return TLine2<T> (l.A() - vect, l.V());
	}
	template <typename T, typename U> void operator -= ( const TLine2<T> & l, const TPoint2<U> & vect )
	{
		l = l - vect;
	}

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TLine2<T> & l )
	{
		return ( os << "(" << l.A() << "+t" << l.V() << ")" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TLine2<T> & l )
	{
		char dummy;
		return ( is >> dummy >> l.A() >> dummy >> dummy >> l.V() >> dummy ) ;
	}

	template <typename T, typename U> T * operator << ( T * data, const TLine2<U> & l )
	{
		return ( data << l.A() << l.V() );
	}

	template <typename T, typename U> T * operator >> ( T * data, TLine2<U> & l )
	{
		return ( data >> l.A() >> l.V() );
	}

	/* ----- typedefs ----- */
	typedef TLine2<double> Line2d;
	typedef TLine2<float> Line2f;
	typedef Line2d Line2;

}; // namespace Lg

#endif //LgLine2_hpp
