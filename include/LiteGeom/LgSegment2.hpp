/** @file
 Light 2D segment class for use in MATIS: Lg::Segment2(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgSegment2_hpp
#define LgSegment2_hpp

#include "LgLine2.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 2D segment class for use in MATIS: Lg::Segment2(d, f)
	/** Defined by their extremities Source (first, A) and Target (second, B)*/
	template <typename T> class TSegment2 : public std::pair< TPoint2<T>, TPoint2<T> >
	{
		typedef std::pair< TPoint2<T>, TPoint2<T> > mother;
	public:
		typedef T coord_type;

		/* ----- constructors ----- */
		TSegment2(const T & ax, const T & ay, const T & bx, const T & by):
		mother(TPoint2<T>(ax,ay), TPoint2<T>(bx,by)){}
		TSegment2(TPoint2<T> A=TPoint2<T>(), TPoint2<T> B=TPoint2<T>()): mother(A, B){}
		TSegment2(T* data): mother(TPoint2<T>(data), TPoint2<T>(data+2)){}
		TSegment2(T** data): mother(TPoint2<T>(data[0]), TPoint2<T>(data[1])){}

		/* ----- acces ----- */
		inline TPoint2<T> Source() const {return mother::first;}
		inline TPoint2<T> Target() const {return mother::second;}
		inline TPoint2<T> & Source() {return mother::first;}
		inline TPoint2<T> & Target() {return mother::second;}

		/// type conversion
		template <typename U> TSegment2<U> Convert() const
		{
			return TSegment2<U>(Source().Convert(), Target().Convert());
		}

		/* ----- modifiers ----- */
		inline void SetSource(TPoint2<T> src){mother::first=src;}
		inline void SetTarget(TPoint2<T> trgt){mother::second=trgt;}
		inline void SetSourceAndTarget(TPoint2<T> src, TPoint2<T> trgt){mother::first=src; mother::second=trgt;}

		/* ----- shortcuts ----- */
		inline TPoint2<T> & A() {return mother::first;}
		inline TPoint2<T> & B() {return mother::second;}
		inline TPoint2<T> A() const {return mother::first;}
		inline TPoint2<T> B() const {return mother::second;}
		inline void SetA(TPoint2<T> A){mother::first=A;}
		inline void SetB(TPoint2<T> B){mother::second=B;}
		inline void SetAB(TPoint2<T> A, TPoint2<T> B){mother::first=A; mother::second=B;}

		/* ----- convenience ----- */
		inline TPoint2<T> Vector() const {return B()-A();}
		inline TLine2<T> Line() const {return TLine2<T>(A(),B()-A());}

		/* ----- geometry ----- */
		inline TPoint2<T> Middle() const {return 0.5*(A()+B());}
		inline T Length2() const {return (B()-A()).Norm2();}
		inline T Length() const {return (B()-A()).Norm();}

	};

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TSegment2<T> & s1, const TSegment2<U> & s2 )
	{
		return ( s1.A() == s2.A() && s1.B() == s2.B() );
	}
	template <typename T, typename U> bool operator != ( const TSegment2<T> & s1, const TSegment2<U> & s2 )
	{
		return ( s1.A() != s2.A() || s1.B() != s2.B() );
	}
	template <typename T, typename U> TSegment2<T> operator + ( const TSegment2<T> & s, const TPoint2<U> & vect )
	{
		return TSegment2<T> (s.A() + vect, s.B() + vect);
	}
	template <typename T, typename U> void operator += ( const TSegment2<T> & s, const TPoint2<U> & vect )
	{
		s = s + vect;
	}
	template <typename T, typename U> TSegment2<T> operator - ( const TSegment2<T> & s, const TPoint2<U> & vect )
	{
		return TSegment2<T> (s.A() - vect, s.B() - vect);
	}
	template <typename T, typename U> void operator -= ( const TSegment2<T> & s, const TPoint2<U> & vect )
	{
		s = s - vect;
	}
        template <class T> TSegment2<T> operator * ( const T & t, const TSegment2<T> & s )
        {
            return TSegment2<T> (t * s.A(), t * s.B());
        }
        template <class T> TSegment2<T> operator / ( const TSegment2<T> & s, const T & t )
        {
            T t_inv = 1./t;
            return TSegment2<T> (t_inv * s.A(), t_inv * s.B());
        }
        template <class T> TSegment2<T> operator + ( const TSegment2<T> & s1, const TSegment2<T> & s2 )
        {
            return TSegment2<T> (s1.A() + s2.A(), s1.B() + s2.B());
        }
        template <class T> TSegment2<T> operator - ( const TSegment2<T> & s1, const TSegment2<T> & s2 )
        {
            return TSegment2<T> (s1.A() - s2.A(), s1.B() - s2.B());
        }

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TSegment2<T> & s )
	{
		return ( os << "[" << s.A() << ";" << s.B() << "]" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TSegment2<T> & s )
	{
		char dummy;
		return ( is >> dummy >> s.A() >> dummy >> s.B() >> dummy ) ;
	}
	template <typename T, typename U> T * operator << ( T * data, const TSegment2<U> & s )
	{
		return ( data << s.A() << s.B() );
	}
	template <typename T, typename U> T * operator >> ( T * data, TSegment2<U> & s )
	{
		return ( data >> s.A() >> s.B() );
	}

	/* ----- typedefs ----- */
	typedef TSegment2<double> Segment2d;
	typedef TSegment2<float> Segment2f;
	typedef Segment2d Segment2;

}; // namespace Lg

#endif //LgSegment2_hpp
