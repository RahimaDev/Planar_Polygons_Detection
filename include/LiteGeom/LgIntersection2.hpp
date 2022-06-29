/** @file
 Light 2D intersections for use in MATIS (fast but no degenerate cases handling).
 Tests if a primitive intersects another.\n
 If a third arg is given, it is used to return the result of the intersection.\n
Author: Bruno Vallet 30/11/2009
*/

#ifndef LgIntersection2_hpp
#define LgIntersection2_hpp

/// namespace for all LiteGeom objects
namespace Lg {

#ifdef LgLine2_hpp
	/// Line2/Line2 intersection
	template<typename T> bool Intersects( const TLine2<T> & l0, const TLine2<T> & l1 )
	{
		return ((l0.Vector()^l1.Vector())!=0.);
	}
	/// x coord of a Line2/Line2 intersection
	template<typename T> T xInter ( const TLine2<T> & l0, const TLine2<T> & l1)
	{
		TPoint2<T> e0=l0.Vector(), e1 = l1.Vector(), v01 = l1.A() - l0.A();
		T det = e1^e0;
		if ( det ) {
			return (e1^v01)/det;
		}
		return std::numeric_limits<T>::max();
	}
	/// Line2/Line2 intersection
	template<typename T> bool Intersects(const TLine2<T> & l0, const TLine2<T> & l1, TPoint2<T> & P)
	{
		T x = xInter(l0, l1);
		P = l0.A() + x*l0.Vector();
		return (x != std::numeric_limits<T>::max());
	}
	/// Line2/Line2 intersection
	template<typename T> TPoint2<T> Intersection(const TLine2<T> & l0, const TLine2<T> & l1)
	{
		return l0.A() + xInter(l0, l1)*l0.Vector();
	}
#endif // LgLine2_hpp

#ifdef LgSegment2_hpp
	/// x coord of a Segment2/Line2 intersection
	template<typename T> T xInter ( const TSegment2<T> & s, const TLine2<T> & l)
	{
		return xInter<T>(s.Line(), l);
	}
	/// x coord of a Line2/Segment2 intersection
	template<typename T> T xInter ( const TLine2<T> & l, const TSegment2<T> & s)
	{
		return xInter<T>(l, s.Line());
	}
	/// x coord of a Segment2/Segment2 intersection
	template<typename T> T xInter ( const TSegment2<T> & s0, const TSegment2<T> & s1)
	{
		return xInter<T>(s0.Line(), s1.Line());
	}
	/// Segment2/Line2 intersection
	template<typename T> bool Intersects(const TSegment2<T> & s, const TLine2<T> & l)
	{
		T x = xInter(s, l);
		return (x > T(0) && x < T(1));
	}
	/// Line2/Segment2 intersection
	template<typename T> bool Intersects(const TLine2<T> & l, const TSegment2<T> & s)
	{
		return Intersects<T>(s, l);
	}
	/// Segment2/Line2 intersection
	template<typename T> bool Intersects(const TSegment2<T> & s, const TLine2<T> & l, TPoint2<T> & I)
	{
		T x = xInter(s, l);
		I = s.A() + x*s.Vector();
		return (x > T(0) && x < T(1));
	}
	/// Line2/Segment2 intersection
	template<typename T> bool Intersects(const TLine2<T> & l, const TSegment2<T> & s, TPoint2<T> & I)
	{
		return Intersects<T>(s, l, I);
	}
	/// Segment2/Segment2 intersection
	template<typename T> bool Intersects(const TSegment2<T> & s0, const TSegment2<T> & s1)
	{
		T x0 = xInter(s0, s1);
		if(x0>0. && x0<1.) {
			T x1 = xInter(s1, s0);
			return (x1>0. && x1<1.);
		}
		return false;
	}
	/// Segment2/Segment2 intersection
	template<typename T> bool Intersects(const TSegment2<T> & s0, const TSegment2<T> & s1, TPoint2<T> & I)
	{
		T x0 = xInter(s0, s1);
		I = s0.A() + x0*s0.Vector();
		if(x0>0. && x0<1.) {
			T x1 = xInter(s1, s0);
			return (x1>0. && x1<1.);
		}
		return false;
	}
#endif // LgSegment2_hpp

#ifdef LgTriangle2_hpp
	/// Triangle2/(Point2/Line2/Segment2) intersection
	template<typename T, class G> bool Intersects(TTriangle2<T> const & Tri, G const & g)
	{
		for (size_t i = 0; i < 3; i++) {
			if(Intersects(g, Tri.Segment(i))) return true;
		}
		return false;
	}
	/// Triangle2/(Point2/Line2/Segment2) intersections appended to vI
	template<typename T, class G> bool Intersects(TTriangle2<T> const & Tri, G const & g,
		std::vector< TPoint2<T> > & vI)
	{
		bool ret = false;
		for (size_t i = 0; i < 3; i++) {
			TPoint2<T> I;
			if(Intersects(g, Tri.Segment(i), I)) {vI.push_back(I); ret = true;}
		}
		return ret;
	}
#endif // LgTriangle2_hpp

#ifdef LgPolygon2_hpp
	/// Polygon2/(Point2/Line2/Segment2) intersection
	template<typename T, class G> bool Intersects(TPolygon2<T> const & Poly, G const & g)
	{
		if(Poly.empty()) return false;
		for (size_t i = 0; i < Poly.Size(); i++) {
			if(Intersects(g, Poly.Segment(i))) return true;
		}
		return false;
	}
	/// (Point2/Line2/Segment2)/Polygon2 intersections appended to vI
	template<typename T, class G> bool Intersects(TPolygon2<T> const & Poly, G const & g,
		std::vector< TPoint2<T> > & vI)
	{
		if(Poly.empty()) return false;
		bool ret = false;
		for (size_t i = 0; i < Poly.Size(); i++) {
			TPoint2<T> I;
			if(Intersects(g, Poly.Segment(i), I)) {vI.push_back(I); ret = true;}
		}
		return ret;
	}
#endif // LgPolygon2_hpp

}; // namespace Lg

#endif // LgIntersection2_hpp
