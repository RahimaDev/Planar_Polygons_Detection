/** @file
 * Light 2D bounding box class for use in MATIS: Lg::Bbox2(d, f).
 * Author: Bruno Vallet 09/2010
*/

#ifndef LgBbox2_hpp
#define LgBbox2_hpp

#include "LgPoint2.hpp"
#include <limits.h>

/// namespace for all LiteGeom objects
namespace Lg {

    /// Light 2D bbox class for use in MATIS: Lg::Bbox2(d, f)
    /** Defined by a normal (first, N) and a signed distance to origin (second, d) */
    template <class T> class TBbox2 : public std::pair< TPoint2<T>, TPoint2<T> >
    {
        typedef std::pair< TPoint2<T>, TPoint2<T> > mother;
    public:
        typedef T coord_type;

        inline void Order(T & v1, T & v2) {if(v1>v2) {T tmp = v1; v1=v2; v2=tmp;}}
        inline void Orient()
        {
            Order(mother::first.x(), mother::second.x());
            Order(mother::first.y(), mother::second.y());
        }
        /// default constructor (empty bbox)
        TBbox2()
        {
            T max = std::numeric_limits<T>::max();
            T min = (std::numeric_limits<T>::is_integer ? std::numeric_limits<T>::min() : -std::numeric_limits<T>::max());
            mother::first = TPoint2<T>(max,max);
            mother::second = TPoint2<T>(min,min);
        }
        /// constructor from 1 value
        TBbox2(const T & val):
        mother(TPoint2<T>(val,val), TPoint2<T>(val,val)) {}

        /// constructor from 2 values
        TBbox2(const T & x, const T & y):
        mother(TPoint2<T>(x,y), TPoint2<T>(x,y)) {}

        /// constructor from 4 values
        TBbox2(const T & xmin, const T & ymin, const T & xmax, const T & ymax):
        mother(TPoint2<T>(xmin,ymin), TPoint2<T>(xmax,ymax)) {Orient();}

        /// constructor from 1 point
        TBbox2(const TPoint2<T> & P):mother(P, P) {}

        /// constructor from 2 points
        TBbox2(const TPoint2<T> & Pmin, const TPoint2<T> & Pmax):mother(Pmin, Pmax) {Orient();}

        /* ----- access ----- */
        inline TPoint2<T> & Min() {return mother::first;}
        inline TPoint2<T> Min() const {return mother::first;}
        inline TPoint2<T> & Max() {return mother::second;}
        inline TPoint2<T> Max() const {return mother::second;}

        inline T & Xmin() {return mother::first.X();}
        inline T Xmin() const {return mother::first.X();}
        inline T & Xmax() {return mother::second.X();}
        inline T Xmax() const {return mother::second.X();}

        inline T & Ymin() {return mother::first.Y();}
        inline T Ymin() const {return mother::first.Y();}
        inline T & Ymax() {return mother::second.Y();}
        inline T Ymax() const {return mother::second.Y();}

        /* ----- modifiers ----- */
        inline void AddPoint(const TPoint2<T> & P)
        {
            if(P.x() < Xmin()) Xmin() = P.x();
            else if(P.x() > Xmax()) Xmax() = P.x();
            if(P.y() < Ymin()) Ymin() = P.y();
            else if(P.y() > Ymax()) Ymax() = P.y();
        }
        inline void AddBbox(const TBbox2<T>& bbx)
        {
            if(bbx.Xmin() < Xmin()) Xmin() = bbx.Xmin();
            if(bbx.Xmax() > Xmax()) Xmax() = bbx.Xmax();
            if(bbx.Ymin() < Ymin()) Ymin() = bbx.Ymin();
            if(bbx.Ymax() > Ymax()) Ymax() = bbx.Ymax();
        }
        inline bool Contains(const TPoint2<T> & P) const
        {
            return P.x() > Xmin() && P.x() < Xmax() && 
                P.y() > Ymin() && P.y() < Ymax();
        }
        inline TPoint2<T> Center() const {return 0.5*(Min() + Max());}
        inline TPoint2<T> Size() const {return (Max() - Min());}
        inline void Translate(const TPoint2<T> & P) {Min() += P; Max()+= P;}
        inline void AddMargin(const TPoint2<T> & P) {Min() -= P; Max()+= P;}
        inline void AddMargin(const T & Margin) {AddMargin(TPoint2<T>(Margin));}

    };

    /* ----- algebraic operators ----- */
    template <class T> bool operator == ( const TBbox2<T> & bbx1, const TBbox2<T> & bbx2 )
    {
        return ( bbx1.Min() == bbx2.Min() && bbx1.Max() == bbx2.Max() );
    }
    template <class T> bool operator != ( const TBbox2<T> & bbx1, const TBbox2<T> & bbx2 )
    {
        return ( bbx1.Min() != bbx2.Min() || bbx1.Max() != bbx2.Max() );
    }
    template <class T> TBbox2<T> operator + ( const TBbox2<T> & bbx1, const TBbox2<T> & bbx2 )
    {
        TBbox2<T> & ret = bbx1;
        ret.AddBbox(bbx2);
        return ret;
    }
    template <class T> void operator += ( TBbox2<T> & bbx1, const TBbox2<T> & bbx2 )
    {
        bbx1.AddBbox(bbx2);
    }
    template <class T> TBbox2<T> operator + ( const TBbox2<T> & bbx, const TPoint2<T> & vect )
    {
        return TBbox2<T> (bbx.Min() + vect, bbx.Max() + vect);
    }
    template <class T> void operator += ( TBbox2<T> & bbx, const TPoint2<T> & vect )
    {
        bbx.Translate(vect);
    }
    template <class T> TBbox2<T> operator - ( const TBbox2<T> & bbx, const TPoint2<T> & vect )
    {
        return TBbox2<T> (bbx.Min() - vect, bbx.Max() - vect);
    }
    template <class T> void operator -= ( TBbox2<T> & bbx, const TPoint2<T> & vect )
    {
        bbx.Translate(-vect);
    }

    /* ----- stream operators ----- */
    template <typename T> std::ostream& operator << ( std::ostream &os, const TBbox2<T> & bbx )
    {
        return ( os << "[" << bbx.Min() << "," << bbx.Max() << "]" ) ;
    }
    template <typename T> std::istream& operator >> ( std::istream &is, TBbox2<T> & bbx )
    {
        char dummy;
        return ( is >> dummy >> bbx.Min() >> dummy >> bbx.Max() >> dummy ) ;
    }

    template <typename T, typename U> T * operator << ( T * data, const TBbox2<U> & bbx )
    {
        return ( data << bbx.Min() << bbx.Max() );
    }

    template <typename T, typename U> T * operator >> ( T * data, TBbox2<U> & bbx )
    {
        return ( data >> bbx.Min() >> bbx.Max() );
    }

    /* ----- typedefs ----- */
    typedef TBbox2<double> Bbox2d;
    typedef TBbox2<float> Bbox2f;
    typedef Bbox2d Bbox2;

}; // namespace Lg

#endif //LgBbox2_hpp

