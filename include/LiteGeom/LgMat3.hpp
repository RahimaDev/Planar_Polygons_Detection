/** @file
 Light 3D matrix class for use in MATIS: Lg::Mat3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgMat3_hpp
#define LgMat3_hpp

#include "LgPoint3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

	/// Light 3D matrix class for use in MATIS: Lg::Mat3(d, f)
	/**
	Represented as std::vector< TPoint3<T> >\n
	Implementation choice: each TPoint3<T> is a line\n
	Reasons: Easier Matrix/Point multiplications, Easier printing,
	coherence with T[i=line][j=column] notation\n
	(xx, xy, xz)\n
	(yx, yy, yz)\n
	(zx, zy, zz)
	*/
	template <class T> class TMat3 : public std::vector< TPoint3<T> >
	{
		typedef std::vector< TPoint3<T> > mother;
	public:
		/* ----- constructors ----- */
		TMat3 ( const T & val = T(0))
		{
			for(size_t i = 0; i < 3; i++) mother::push_back(TPoint3<T>(val, val, val));
		}
		TMat3 ( const TPoint3<T> & X, const TPoint3<T> & Y, const TPoint3<T> & Z )
		{
			mother::push_back(X);
			mother::push_back(Y);
			mother::push_back(Z);
		}
		TMat3 ( const T & xx, const T & xy, const T & xz,
			const T & yx, const T & yy, const T & yz,
			const T & zx, const T & zy, const T & zz )
		{
			mother::push_back(TPoint3<T>(xx, xy, xz));
			mother::push_back(TPoint3<T>(yx, yy, yz));
			mother::push_back(TPoint3<T>(zx, zy, zz));
		}
		TMat3 ( T* data )
		{
			for(size_t i = 0; i < 3; i++) mother::push_back(TPoint3<T>(data + 3*i));
		}
		TMat3 ( T** data )
		{
			for(size_t i = 0; i < 3; i++) mother::push_back(TPoint3<T>(data[i]));
		}

		/* ----- access ----- */
		inline T m(size_t i, size_t j) const {return mother::at(i)[j];}
		inline T operator ()(size_t i, size_t j) const {return mother::at(i)[j];}
		inline T & operator ()(size_t i, size_t j) {return mother::at(i)[j];}
		inline TPoint3<T> Line(size_t i) const {return mother::at(i);}
		inline TPoint3<T> Column(size_t j) const {
			return TPoint3<T>( m(0, j), m(1, j), m(2, j) );
		}
		inline TPoint3<T> operator ()(size_t i) const {return Line(i);}

		inline TPoint3<T> X() const {return Line(0);}
		inline TPoint3<T> Y() const {return Line(1);}
		inline TPoint3<T> Z() const {return Line(2);}
		inline TPoint3<T> & X() {return Line(0);}
		inline TPoint3<T> & Y() {return Line(1);}
		inline TPoint3<T> & Z() {return Line(2);}

		inline T xx() const {return m(0, 0);}
		inline T xy() const {return m(0, 1);}
		inline T xz() const {return m(0, 2);}
		inline T yx() const {return m(1, 0);}
		inline T yy() const {return m(1, 1);}
		inline T yz() const {return m(1, 2);}
		inline T zx() const {return m(2, 0);}
		inline T zy() const {return m(2, 1);}
		inline T zz() const {return m(2, 2);}
		inline T & xx() {return m(0, 0);}
		inline T & xy() {return m(0, 1);}
		inline T & xz() {return m(0, 2);}
		inline T & yx() {return m(1, 0);}
		inline T & yy() {return m(1, 1);}
		inline T & yz() {return m(1, 2);}
		inline T & zx() {return m(2, 0);}
		inline T & zy() {return m(2, 1);}
		inline T & zz() {return m(2, 2);}

		/// type conversion
		template <typename U> TMat3<U> Convert() const
		{
			return TMat3<U>(X().Convert(), Y().Convert(), Z().Convert());
		}

		/* ----- operators ----- */
		inline bool Equal(const TMat3<T> & M) const
		{
			return Line(0)==M(0) && Line(1)==M(1) && Line(2)==M(2);  
		}
		inline T Norm2() const
		{
			return Line(0).Norm2()+Line(1).Norm2()+Line(2).Norm2();
		}
		inline T Norm(){return sqrt(Norm2());}
#ifdef Opposite
#undef Opposite
#endif // Opposite
		inline TMat3<T> Opposite() const
		{
			return TMat3<T>(-Line(0), -Line(1), -Line(2));
		}
		inline TMat3<T> Transposed() const
		{
			return TMat3<T>(Column(0), Column(1), Column(2) );
		}
		inline TMat3<T> Add(const TMat3<T> & M) const {
			return TMat3<T>(Line(0)+M(0), Line(1)+M(1), Line(2)+M(2));
		}
		inline TMat3<T> Sub(const TMat3<T> & M) const
		{
			return TMat3<T>(Line(0)-M(0), Line(1)-M(1), Line(2)-M(2));
		}
		inline TMat3<T> Multiply(const T & a) const
		{
			return TMat3<T>(Line(0)*a, Line(1)*a, Line(2)*a);
		}
		inline TPoint3<T> Multiply(const TPoint3<T> & P) const
		{
			return TPoint3<T>(Line(0)*P, Line(1)*P, Line(2)*P);
		}
		inline TMat3<T> Multiply(const TMat3<T> & M) const
		{
			return TMat3<T>( Multiply(M.Column(0)), Multiply(M.Column(1)), Multiply(M.Column(2)) ).Transposed();
		}

		inline T Trace() {return m(0, 0) + m(1, 1) + m(2, 2);}
		inline T Determinant()
		{
			return m(0,0)*m(1,1)*m(2,2)+m(0,1)*m(1,2)*m(2,0)+m(0,2)*m(1,0)*m(2,1)-
				m(0,0)*m(2,1)*m(1,2)-m(0,1)*m(2,2)*m(1,0)-m(0,2)*m(2,0)*m(1,1);
		}
		inline TMat3<T> Comat()
		{
			return TMat3<T>(m(1,1)*m(2,2)-m(1,2)*m(2,1), m(2,1)*m(0,2)-m(2,2)*m(0,1), m(0,1)*m(1,2)-m(0,2)*m(1,1),
				m(1,2)*m(2,0)-m(1,0)*m(2,2), m(2,2)*m(0,0)-m(2,0)*m(0,2), m(0,2)*m(1,0)-m(0,0)*m(1,2),
				m(1,0)*m(2,1)-m(1,1)*m(2,0), m(2,0)*m(0,1)-m(2,1)*m(0,0), m(0,0)*m(1,1)-m(0,1)*m(1,0) );
		}
		inline TMat3<T> Inverse() {return Comat().Multiply(1./Determinant());}

		/* ----- modifiers ----- */
		inline void Setij(size_t i, size_t j, const T & val) { m(i,j)=T(val);}
	};

	/* ----- operators ----- */
	template <class T> bool operator == ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return A.Equal(B);
	}
	template <class T> bool operator != ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return !(A.Equal(B));
	}
	template <class T> TMat3<T> operator + ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return A.Add(B);
	}
	template <class T> void operator += ( TMat3<T> & A, const TMat3<T> & B )
	{
		A = A + B;
	}
	template <class T> TMat3<T> operator - ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return A.Sub(B);
	}
	template <class T> void operator -= ( TMat3<T> & A, const TMat3<T> & B )
	{
		A = A - B;
	}
	template <class T> TMat3<T> operator - ( const TMat3<T> & A )
	{
		return A.Opposite();
	}
	template <class T> TMat3<T> operator * ( const T & a, const TMat3<T> & B )
	{
		return B.Multiply(a);
	}
	template <class T> TMat3<T> operator * ( const TMat3<T> & A, const T & b )
	{
		return A.Multiply(b);
	}
	template <class T> TMat3<T> operator / ( const TMat3<T> & A, const T & b )
	{
		return A * ( T ( 1 ) /b );
	}
	template <class T> TPoint3<T> operator * ( const TMat3<T> & A, const TPoint3<T> & P )
	{
		return A.Multiply(P);
	}
	template <class T> TMat3<T> operator * ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return A.Multiply(B);
	}
	template <class T> TMat3<T> operator / ( const TMat3<T> & A, const TMat3<T> & B )
	{
		return A * B.Inverse();
	}
	template <class T> std::ostream& operator << ( std::ostream &os, const TMat3<T> & M )
	{
		return ( os << M.Line(0) << std::endl << M.Line(1) << std::endl << M.Line(2) ) ;
	}
	/// Matrix of the projection on a vector: M(V)P = (P.V)V
	template <class T> TMat3<T> ProjectionMatrix(const TPoint3<T> & V)
	{
		return TMat3<T>(V.x()*V, V.y()*V, V.z()*V);
	}
	/// Matrix of the cross product with a vector: M(V)P = V^P
	template <class T> TMat3<T> CrossProductMatrix(const TPoint3<T> & V)
	{
		return TMat3<T>(T(0)  , -V.z(), V.y() ,
			V.z() , T(0)  , -V.x(),
			-V.y(), V.x() , T(0) );
	}

	// typedefs
	typedef TMat3<double> Mat3d;
	typedef TMat3<float> Mat3f;
	typedef Mat3d Mat3;

}; // namespace Lg

#endif //LgMat3_hpp
