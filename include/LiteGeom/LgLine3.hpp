/** @file
 Light 3D line class for use in MATIS: Lg::Line3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgLine3_hpp
#define LgLine3_hpp

#include "LgPoint3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 3D line class for use in MATIS: Lg::Line3(d, f)
	/** defined by a Point (first, A) and a Vector (second, V) */
	template <class T> class TLine3 : public std::pair< TPoint3<T>, TPoint3<T> >
	{
		typedef std::pair< TPoint3<T>, TPoint3<T> > mother;
	public:
		typedef T coord_type;

		/* ----- constructors ----- */
		TLine3(const T & ax, const T & ay, const T & az,
			const T & vx, const T & vy, const T & vz):
		mother(TPoint3<T>(ax,ay,az), TPoint3<T>(vx,vy,vz)){}
		TLine3(TPoint3<T> A = TPoint3<T>(), TPoint3<T> V = TPoint3<T>()): mother(A, V){}

		/* ----- access ----- */
		inline TPoint3<T> & A() {return mother::first;}
		inline TPoint3<T> A() const {return mother::first;}
		inline TPoint3<T> Point() const {return mother::first;}
		inline TPoint3<T> & Point() {return mother::first;}
		inline TPoint3<T> B() const {return mother::first+mother::second;}
		inline TPoint3<T> & V() {return mother::second;}
		inline TPoint3<T> V() const {return mother::second;}
		inline TPoint3<T> & Vector() {return mother::second;}
		inline TPoint3<T> Vector() const {return mother::second;}

		/// type conversion
		template <typename U> TLine3<U> Convert() const
		{
			return TLine3(A().Convert(), V().Convert());
		}

		/* ----- modifiers ----- */
		inline void SetA(TPoint3<T> A){mother::first=A;}
		inline void SetPoint(TPoint3<T> pt){mother::first=pt;}
		inline void SetV(TPoint3<T> V){mother::second=V;}
		inline void SetVector(TPoint3<T> vect){mother::second=vect;}
		inline void SetAV(TPoint3<T> A, TPoint3<T> V){mother::first=A; mother::second=V;}
		inline void SetPointAndVector(TPoint3<T> pt, TPoint3<T> vect){mother::first=pt; mother::second=vect;}
		inline void Translate(const TPoint3<T> & vect){mother::first.Translate(vect);}
	};

	/* ----- algebraic operators ----- */
	template <class T> bool operator == ( const TLine3<T> & l1, const TLine3<T> & l2 )
	{
		return ( l1.A() == l2.A() && l1.V() == l2.V() );
	}
	template <class T> bool operator != ( const TLine3<T> & l1, const TLine3<T> & l2 )
	{
		return ( l1.A() != l2.A() || l1.V() != l2.V() );
	}
	template <class T> TLine3<T> operator + ( const TLine3<T> & l, const TPoint3<T> & vect )
	{
		return TLine3<T> (l.A() + vect, l.V());
	}
	template <class T> void operator += ( const TLine3<T> & l, const TPoint3<T> & vect )
	{
		l = l + vect;
	}
	template <class T> TLine3<T> operator - ( const TLine3<T> & l, const TPoint3<T> & vect )
	{
		return TLine3<T> (l.A() - vect, l.V());
	}
	template <class T> void operator -= ( const TLine3<T> & l, const TPoint3<T> & vect )
	{
		l = l - vect;
	}

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TLine3<T> & l )
	{
		return ( os << "(" << l.A() << "+t" << l.V() << ")" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TLine3<T> & l )
	{
		char dummy;
		return ( is >> dummy >> l.A() >> dummy >> dummy >> l.V() >> dummy ) ;
	}

	template <typename T, typename U> T * operator << ( T * data, const TLine3<U> & l )
	{
		return ( data << l.A() << l.V() );
	}

	template <typename T, typename U> T * operator >> ( T * data, TLine3<U> & l )
	{
		return ( data >> l.A() >> l.V() );
	}

	/* ----- typedefs ----- */
	typedef TLine3<double> Line3d;
	typedef TLine3<float> Line3f;
	typedef Line3d Line3;

}; // namespace Lg

#endif //LgLine3_hpp
