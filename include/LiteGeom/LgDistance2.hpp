/** @file
 * Light 2D distance class for use in MATIS.
 * As in all Lg lib, type of the first arg is kept\n
 * For each function we give the numbers of (+-,*,/,<>!=) operators used in worst case\n
 * Author: Bruno Vallet 30/11/2009
*/

#ifndef LgDistance2_hpp
#define LgDistance2_hpp

#include <math.h>
#include "LgPoint2.hpp"
#include "LgLine2.hpp"
#include "LgSegment2.hpp"

/// namespace for all LiteGeom objects
namespace Lg {


	/// Point2/Point2 (3,2,0,0)
	template <typename T, typename U> T SquaredDistance( const TPoint2<T> & P0, const TPoint2<U> & P1 )
	{
		return (P1-P0).Norm2();
	}

	/// Line2/Point2 (4,5,1,0)
	template <typename T, typename U> T SquaredDistance( const TLine2<T> & l, const TPoint2<U> & P )
	{
		T y = l.Vector()^(l.A() - P); // 3,2,0,0
		return (y*y)/l.Vector().Norm2(); // 1,3,1,0
	}
	/// Point2/Line2 (4,5,1,0)
	template <typename T, typename U> T SquaredDistance( const TPoint2<T> & P, const TLine2<U> & l )
	{
		T y = (P-l.A())^l.Vector();
		return (y*y)/T(l.Vector().Norm2());
	}
	/// Line2/Line2 (5,7,1,1)
	template <typename T, typename U> T SquaredDistance( const TLine2<T> & l0, const TLine2<U> & l1)
	{
		T det = l0.Vector()^l1.Vector(); // 1,2,0,0
		if(det) return T(0); // 0,0,0,1
		return SquaredDistance( l0.A(), l1 ); // 4,5,1,0
	}

	/// Segment2/Point2 (9,8,1,2)
	template <typename T, typename U> T SquaredDistance( const TSegment2<T> & s, const TPoint2<U> & P )
	{
		TPoint2<T> v = s.Vector(); // 2,0,0,0
		TPoint2<T> PA = s.A()-P; // 2,0,0,0
		T x = PA*v, x2 = x*x; // 1,3,0,0
		T y = PA^v, y2 = y*y; // 1,3,0,0
		T n2 = v.Norm2(); // 1,2,0,0
		if(x<T(0)) { // 0,0,0,1
			x2 -= n2; // 1,0,0,0
			if(x2<T(0)) return y2/n2; // 0,0,0,1 (not worst)
		}
		return (x2 + y2)/n2; // 1,0,1,0
	}
	/// Point2/Segment2 (9,8,1,2)
	template <typename T, typename U> T SquaredDistance( const TPoint2<T> & P, const TSegment2<U> & s )
	{
		TPoint2<U> v = s.Vector();
		TPoint2<T> AP = P-s.A();
		T x = AP*v, x2 = x*x;
		T y = AP^v, y2 = y*y;
		T n2 = T(v.Norm2());
		if(x>T(0)) {
			x2 -= n2;
			if(x2<T(0)) return y2/n2;
		}
		return (x2 + y2)/n2;
	}
	/// Line2/Segment2 (9,8,1,4)
	template <typename T, typename U> T SquaredDistance( const TLine2<T> & l, const TSegment2<U> & s)
	{
		T yA = l.Vector()^(l.A() - s.A()); // 3,2,0,0
		T yB = l.Vector()^(l.A() - s.B()); // 3,2,0,0
		if((yA>0) != (yB>0)) return 0; // 0,0,0,3
		yA*=yA; yB*=yB; // 0,2,0,0
		if(yA>yB) return yB/l.Vector().Norm2(); // 2,2,1,1
		return yA/l.Vector().Norm2();
	}
	/// Segment2/Line2 (9,8,1,4)
	template <typename T, typename U> T SquaredDistance( const TSegment2<T> & s, const TLine2<U> & l)
	{
		T yA = (s.A() - l.A())^l.Vector(); // 3,2,0,0
		T yB = (s.B() - l.A())^l.Vector(); // 3,2,0,0
		if((yA>0) != (yB>0)) return 0; // 0,0,0,3
		yA*=yA; yB*=yB; // 0,2,0,0
		if(yA>yB) return yB/T(l.Vector().Norm2()); // 2,2,1,1
		return yA/T(l.Vector().Norm2());
	}
	/// Segment2/Segment2 (39,32,4,11) TODO: optimize
	template <typename T, typename U> T SquaredDistance( const TSegment2<T> & s0, const TSegment2<U> & s1)
	{
		T ret = SquaredDistance(s0.A(), s1); // 9,8,1,2
		T d = SquaredDistance(s0.B(), s1); // 9,8,1,2
		if(d<ret) ret = d; // 1,0,0,1
		d = SquaredDistance(s0, s1.A()); // 9,8,1,2
		if(d<ret) ret = d; // 1,0,0,1
		d = SquaredDistance(s0, s1.B()); // 9,8,1,2
		if(d<ret) ret = d; // 1,0,0,1
		return ret;
	}

	// TODO: distances involving Triangles and Polygons

#ifndef LgDistance3_hpp
	/// generic square root of a squared distance
	template <class G1, class G2> typename G1::coord_type Distance( const G1 & g1, const G2 & g2 )
	{
		typename G1::coord_type d = SquaredDistance(g1, g2);
		return sqrt(d);
	}
#endif // LgDistance3_hpp

}; // namespace Lg

#endif // LgDistance2_hpp
