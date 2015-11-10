#ifndef HVMC_MATH_H
#define HVMC_MATH_H

#include <cmath>
#include <algorithm>
#include <limits>

typedef char      i8;
typedef short     i16;
typedef int       i32;
typedef long long i64;

typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef unsigned long long u64;

typedef float  f32;
typedef double f64;

struct vec3{
    f32 x, y, z;
};


inline vec3 operator*( vec3 const& v, f32 s )
{
    vec3 result{ v.x * s, v.y * s ,v.z * s};
    return result;
}



inline vec3 operator*( f32 s ,vec3 const& v)
{
    return v*s;
}

inline vec3 operator*( vec3 const& v, vec3 const& w)
{
    vec3 result{ v.x * w.x, v.y * w.y ,v.z * w.z};
    return result;
}


struct vec2
{
    f32 x, y;
};

inline vec2 operator+( vec2 const& v, vec2 const& w )
{
    vec2 result{ v.x + w.x, v.y + w.y };
    return result;
}

inline vec2 operator+=( vec2 & v, vec2 const& w )
{
    v = v + w;
    return v;
}

inline vec2 operator-( vec2 const& v, vec2 const& w )
{
    vec2 result{ v.x - w.x, v.y - w.y };
    return result;
}

inline vec2 operator-( vec2 const& v )
{
    vec2 result{ -v.x, -v.y };
    return result;
}

inline vec2 operator*( vec2 const& v, f32 s )
{
    vec2 result{ v.x * s, v.y * s };
    return result;
}

inline vec2 operator*( f32 s, vec2 const& v )
{
    vec2 result{ v.x * s, v.y * s };
    return result;
}

inline vec2 operator/( vec2 const& v, f32 s )
{
    vec2 result{ v.x /s, v.y / s };
    return result;
}

inline f32 Dot( vec2 const& v, vec2 const& w )
{
    f32 result = v.x * w.x + v.y * w.y;
    return result;
}

inline f32 Length( vec2 const& v )
{
    f32 result = sqrt( v.x * v.x + v.y * v.y );
    return result;
}

inline f32 LengthSquared( vec2 const& v )
{
    f32 result = v.x * v.x + v.y * v.y;
    return result;
}

inline vec2 Normalize( vec2 const& v )
{
    f32 len = Length( v );
    vec2 result{ v.x, v.y };
    if ( len > std::numeric_limits<f32>::epsilon() )
    {
        f32 invLen = f32(1) / len;
        result.x *= invLen;
        result.y *= invLen;
    }
    return result;
}

inline f32 Cross( vec2 const& v, vec2 const& w )
{
    f32 result{ v.x * w.y - v.y * w.x };
    return result;
}

inline vec2 Cross( vec2 const& v, f32 s )
{
    vec2 result{ s * v.y, -s * v.x };
    return result;
}

inline vec2 Cross( f32 s, vec2 const& v )
{
    vec2 result{ -s * v.y, s * v.x };
    return result;
}

inline f32 Distance( vec2 const& v, vec2 const& w )
{
    f32 result = Length( v - w );
    return result;
}

inline vec2 Min( vec2 const& v, vec2 const& w )
{
    vec2 result{ std::min( v.x, w.x ), std::min( v.y, w.y ) };
    return result;
}

inline vec2 Max( vec2 const& v, vec2 const& w )
{
    vec2 result{ std::max( v.x, w.x ), std::max( v.y, w.y ) };
    return result;
}

struct Mat2
{
    f32 m00, m01;
    f32 m10, m11;
};

inline Mat2 Mat2_Zeros()
{
    Mat2 result{ 0.f, 0.f, 0.f, 0.f };
    return result;
}

inline Mat2 Mat2_Identity()
{
    Mat2 result{ 1.f, 0.f, 0.f, 1.f };
    return result;
}

inline Mat2 Mat2_Ones()
{
    Mat2 result{ 1.f, 1.f, 1.f, 1.f };
    return result;
}

inline Mat2 Mat2_FromAngle( f32 radians )
{
    f32 c = std::cos( radians );
    f32 s = std::sin( radians );
    Mat2 result{ c, -s, s, c };
    return result;
}

inline Mat2 Mat2_Transpose( Mat2 const& m )
{
    Mat2 result{ m.m00, m.m10, m.m01, m.m11 };
    return result;
}

#endif
