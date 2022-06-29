/** @file
 Light 3D plane class for use in MATIS: Lg::Plane3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgPlane3_hpp
#define LgPlane3_hpp

#include "LgLine3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 3D plane class for use in MATIS: Lg::Plane3(d, f)
	/** Defined by a normal (first, N) and a signed distance to origin (second, d) */
	template <class T> class TPlane3 : public std::pair< TPoint3<T>, T >
	{
		typedef std::pair< TPoint3<T>, T > mother;
	public:
		typedef T coord_type;

		/// constructor (plane of normal (nx,ny,nz) and signed distance d/||(nx,ny,nz)|| to origin)
		/** The plane equation is OP.N=d, even if N is not normalized */
		TPlane3(const T & nx = T(0), const T & ny = T(0), const T & nz = T(0), const T & d = T(0)):
		mother(TPoint3<T>(nx,ny,nz), d) {}
		/// constructor (plane of normal N and signed distance d/||N|| to origin)
		/** The plane equation is OP.N=d, even if N is not normalized */
		TPlane3(TPoint3<T> N, const T & d = T(0)): mother(N, d){}
		/// constructor (plane of normal N passing through A)
		/** The plane equation is OP.N=OA.N, even if N is not normalized */
		TPlane3(TPoint3<T> N, TPoint3<T> A): mother(N, A*N){}
		/// constructor (plane passing through A,B and C)
		/** The plane equation is OP.N=OA.N, with N=AB^AC */
		TPlane3(TPoint3<T> A, TPoint3<T> B, TPoint3<T> C) {
			mother::first = (B-A)^(C-A);
			mother::second = A*mother::first;
		}
		/* ----- access ----- */
		inline TPoint3<T> & N() {return mother::first;}
		inline TPoint3<T> N() const {return mother::first;}
		inline TPoint3<T> & Normal() {return mother::first;}
		inline TPoint3<T> Normal() const {return mother::first;}
		inline T & d() {return mother::second;}
		inline T d() const {return mother::second;}
		inline T & Dist() {return mother::second;}
		inline T Dist() const {return mother::second;}
		inline T SignedDistanceToOrigin() const {return mother::second;}
		inline TPoint3<T> Point() const {return (d()/(N()*N()))*N();}

        inline TPoint3<T> & Projection(const TPoint3<T> & P) {return P - ((P*N()-d())/(N()*N()))*N();}

		/* ----- modifiers ----- */
		inline void SetN(TPoint3<T> N){mother::first=N;}
		inline void SetNormal(TPoint3<T> normal){mother::first=normal;}
		inline void Setd(T d){mother::second=d;}
		inline void SetDist(T dist){mother::second=dist;}
		inline void SetNd(TPoint3<T> N, T d){mother::first=N; mother::second=d;}
		inline void SetNormalAndDist(TPoint3<T> normal, T dist)
        {mother::first=normal; mother::second=dist;}
		inline void Normalize(){mother::second *= mother::first.Normalize(); }

	};

	/* ----- algebraic operators ----- */
	template <class T> bool operator == ( const TPlane3<T> & p1, const TPlane3<T> & p2 )
	{
		return ( p1.N() == p2.N() && p1.d() == p2.d() );
	}
	template <class T> bool operator != ( const TPlane3<T> & p1, const TPlane3<T> & p2 )
	{
		return ( p1.N() != p2.N() || p1.d() != p2.d() );
	}
	template <class T> TPlane3<T> operator + ( const TPlane3<T> & p, const TPoint3<T> & vect )
	{
		return TPlane3<T> (p.N(), p.d() + (p.N() * vect));
	}
	template <class T> void operator += ( const TPlane3<T> & p, const TPoint3<T> & vect )
	{
		p = p + vect;
	}
	template <class T> TPlane3<T> operator - ( const TPlane3<T> & p, const TPoint3<T> & vect )
	{
		return TPlane3<T> (p.N(), p.d() - (p.N() * vect));
	}
	template <class T> void operator -= ( const TPlane3<T> & p, const TPoint3<T> & vect )
	{
		p = p - vect;
	}

	/* ----- stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TPlane3<T> & p )
	{
		return ( os << "(P." << p.Normal() << "=" << p.d() << ")" ) ;
	}
	template <typename T> std::istream& operator >> ( std::istream &is, TPlane3<T> & p )
	{
		char dummy;
		return ( is >> dummy >> dummy >> dummy >> p.Normal() >> dummy >> p.d() >> dummy ) ;
	}

	template <typename T, typename U> T * operator << ( T * data, const TPlane3<U> & p )
	{
		return ( data << p.Normal() << p.d() );
	}

	template <typename T, typename U> T * operator >> ( T * data, TPlane3<U> & p )
	{
		return ( data >> p.Normal() >> p.d() );
	}

	/* ----- typedefs ----- */
	typedef TPlane3<double> Plane3d;
	typedef TPlane3<float> Plane3f;
	typedef Plane3d Plane3;

}; // namespace Lg

#endif //LgPlane3_hpp
