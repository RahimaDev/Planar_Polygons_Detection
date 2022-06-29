/** @file
 Light 2D polygon classes for use in MATIS: Lg::Polygon2(d, f).
Author: Bruno Vallet 16/11/2009
*/

#ifndef LgPolygon2_hpp
#define LgPolygon2_hpp

#include "LgSegment2.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 2D polygon class for use in MATIS: Lg::Polygon2(d, f)
	/** Defined by a vector of Point2 */
	template <class T> class TPolygon2 : public std::vector< TPoint2<T> >
	{
		typedef std::vector< TPoint2<T> > mother;
	public:
		typedef T coord_type;
		/* ----- constructors ----- */
		// Rem: default is to use () then PushBack(TPoint2<T> P)
		TPolygon2() : mother() {}
		TPolygon2(T* data, size_t n_points) {
			for(size_t i=0; i<n_points; i++) mother::push_back(TPoint2<T>(data + 2*i));
		}
		TPolygon2(T** data, size_t n_points) {
			for(size_t i=0; i<n_points; i++) mother::push_back(TPoint2<T>(*(data + i)));
		}
		TPolygon2(TPoint2<T>* data, size_t n_points) {
			for(size_t i=0; i<n_points; i++) mother::push_back(data[i]);
		}
#ifdef LgTriangle2_hpp
		/// Constructor from a Triangle2 (include LgTriangle2.hpp before LgPolygon2.hpp)
		TPolygon2( const TTriangle2<T> & Tri )
		{
			mother::push_back(Tri.A());
			mother::push_back(Tri.B());
			mother::push_back(Tri.C());
		}
#endif // LgTriangle2_hpp

		/* ----- access to vertices and edges ----- */
		inline size_t Size() const {return mother::size();}
		inline TPoint2<T> & Point(size_t i) {return mother::at(i%Size());}
		inline TPoint2<T> const & Point(size_t i) const {return mother::at(i%Size());}
		inline TSegment2<T> Segment(size_t i) const
		{
            return TSegment2<T>(Point(i%mother::size()),Point((i+1)%mother::size()));
		}
		inline TPoint2<T> operator[](size_t i) const {return Point(i);}
		inline TPoint2<T> & operator[](size_t i) {return Point(i);}
		inline TPolygon2<T> Reversed() const {
			TPolygon2<T> ret;
			for(size_t i = 0; i < Size(); i++) {ret.push_back(Point(Size()-1-i));}
			return ret;
		}

		/// type conversion
		template <typename U> TPolygon2<U> Convert() const
		{
			TPolygon2<T> ret;
			for(size_t i = 0; i < Size(); i++) {ret.push_back(Point(i).Convert());}
			return ret;
		}

		/* ----- modifiers ----- */
		inline void PushBack(TPoint2<T> P) {return mother::push_back(P);}
		inline void AddPoint(TPoint2<T> P) {return mother::push_back(P);}
		inline TPolygon2<T> Reverse() {mother::swap(Reversed());}

		/* ----- geometry ----- */
		inline TPoint2<T> SignedArea() const {
			T a(0);
			for(size_t i = 0; i < Size(); i++) {a += Point(i)^Point(i+1);}
			return 0.5 * a;
		}
		inline T Area() const {
			T a = SignedArea();
			if(a > 0) return a;
			return -a;
		}
		inline TPoint2<T> WeightedG() const {
			TPoint2<T> P(0, 0);
			for(size_t i = 0; i < Size(); i++) {P += Point(i);}
			return P;
		}
		inline TPoint2<T> G() const {
			return WeightedG() / T(Size());
		}
		inline T Perimeter() const
		{
			T ret = 0.;
			for(unsigned int i = 0; i < Size(); i++) ret += (Point(i+1)-Point(i)).Norm();
			return ret;
		}
		inline bool IsConvex(size_t iV) const {
			return (Point(iV)-Point(iV+Size()-1))^(Point(iV+1)-Point(iV)) > 0;
		}
		inline bool IsConvex() const {
			for(size_t iV = 0; iV<Size(); iV++) if(!IsConvex(iV)) return false;
			return true;
		}

	}; // class TPolygon2

	/* ----- algebraic operators ----- */
	// Rem: always keep the type of the first operand
	template <typename T, typename U> bool operator == ( const TPolygon2<T> & p1, const TPolygon2<U> & p2 )
	{
		if(p1.Size() != p2.Size()) return false;
		for(size_t i=0; i<p1.Size(); i++) if(p1[i]!=p2[i]) return false;
		return true;
	}
	template <typename T, typename U> bool operator != ( const TPolygon2<T> & p1, const TPolygon2<U> & p2 )
	{
		return !(p1==p2);
	}
	template <typename T, typename U> TPolygon2<T> operator + ( const TPolygon2<T> & p, const TPoint2<U> & vect )
	{
		TPolygon2<T> ret;
		for(size_t i=0; i<p.Size(); i++) ret.PushBack(p[i] + vect);
		return ret;
	}
	template <typename T, typename U> void operator += ( const TPolygon2<T> & p, const TPoint2<U> & vect )
	{
		p = p + vect;
	}
	template <typename T, typename U> TPolygon2<T> operator - ( const TPolygon2<T> & p, const TPoint2<U> & vect )
	{
		TPolygon2<T> ret;
		for(size_t i=0; i<p.Size(); i++) ret.PushBack(p[i] - vect);
		return ret;
	}
	template <typename T, typename U> void operator -= ( const TPolygon2<T> & p, const TPoint2<U> & vect )
	{
		p = p - vect;
	}

	/* ----- output stream operators ----- */
	template <typename T> std::ostream& operator << ( std::ostream &os, const TPolygon2<T> & p )
	{
		os << "{";
		if(p.Size()) os << p[0];
		for(size_t i=1; i<p.Size(); i++) os << ";" << p[i] ;
		return ( os << "}" );
	}
	template <typename T, typename U> T * operator << ( T * data, const TPolygon2<U> & p )
	{
		for(size_t i=0; i<p.Size(); i++) data << p[i] ;
		return data;
	}

	// TODO: input streams

	/* ----- typedefs ----- */
	typedef TPolygon2<float> Polygon2f;
	typedef TPolygon2<double> Polygon2d;
	typedef Polygon2d Polygon2;

}; // namespace Lg

#endif // LgPolygon2_hpp
