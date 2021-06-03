#ifndef VEC_H
#define VEC_H
#include<cmath>
#include<cstddef>
#include<stdexcept>
#include<iostream>
#include <limits>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#define TOLERANCE 1e-7

// Boost-like compile-time assertion checking
typedef double RealType;
template<bool flag>
struct VEC_STATIC_ASSERT_FAILURE;
template <> struct VEC_STATIC_ASSERT_FAILURE<true>
{
    void operator() () {}
};
#define VEC_STATIC_CHECK(expr) VEC_STATIC_ASSERT_FAILURE<bool(expr)>()
namespace GCL {


/**
 * @brief      向量类
 *
 * @tparam     D     维数
 * @tparam     T     每位数据类型
 * 完整的向量类，支持向量基本运算，以及计算点到直线平面，三角面的投影
 * 包含类0-1区间RGB向量到256RGB向量的转换
 */
template<size_t D, class T = float>
class Vec
{
public:
    //--- class info ---

    /// type of this vector
    typedef Vec<D,T>  vector_type;

    ///the type of the Scalar used in this template
    typedef T value_type;

    /// returns dimension N of the vector
    static int dim()
    {
        return D;
    }

    /// returns dimension of the vector
    static inline size_t size() { return D; }

    static const size_t size_ = D;

    Vec() {for(size_t i=0; i < D; i++) v[i] = (T)0;}
    explicit Vec(const T& x) {for(size_t i=0; i < D; i++) v[i] = x;}
    Vec(const T& x, const T& y)
    {
//        VEC_STATIC_CHECK(D==2);
        v[0] = x; v[1] = y;
    }
    Vec(const T &x, const T &y, const T &z)
    {
        VEC_STATIC_CHECK(D==3);
        v[0] = x; v[1] = y; v[2] = z;
    }
    Vec(const T &x, const T &y, const T &z, const T &w)
    {
        VEC_STATIC_CHECK(D==4);
        v[0] = x; v[1] = y; v[2] = z; v[3] = w;
    }

    template<typename NT>
    Vec(const Vec<D,NT>& other)
    {
        for(int i=0; i < D; i++)
        {
            v[i] = (T)other[i];
        }
    }

    template<typename NT>
    Vec(const NT* pv)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] = (T)pv[i];
        }
    }
    const T* data() const { return v;}
    const T* getPtr() const { return v;}
    T& operator [] (size_t i)
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    T& operator [] (int i)
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    const T& operator [] (size_t i) const
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    const T& operator [] (int i) const
    {
        if(i>=D) throw std::out_of_range("Vec::[]");
        return v[i];
    }
    T& at(size_t i)
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    T& at(int i)
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    const T& at(size_t i) const
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }
    const T& at(int i) const
    {
        if(i>=D) throw std::out_of_range("Vec::at");
        return v[i];
    }

    /**
     * @brief      单位化该向量
     *
     * @return     单位化后的向量
     */
    T Normalize()
    {
        T len = length();
        if(len <= (T)TOLERANCE)
        {
            return len;
        }
        for(size_t i=0; i < D; i++)
        {
            v[i] /= len;
        }
        return (T)1;
    }

    /**
     * @brief      向量的平方范数
     *
     * @return     向量的平方范数
     */
    T sqrnorm() const
    {
        return length2();
    }

    T norm() const
    {
        return length();
    }
    Vec<D,T>& normalize()
    {

        T len = length();
        if(len > (T)TOLERANCE)
        {
            for(size_t i=0; i < D; i++)
            {
                v[i] = v[i] / len;
            }
        }
        return *this;
    }
    Vec<D,T> normalize() const
    {
        Vec<D,T> vout;
        T len = length();
        if(len > (T)TOLERANCE)
        {
            for(size_t i=0; i < D; i++)
            {
                vout[i] = v[i] / len;
            }
        }
        return vout;
    }
    Vec<D,T>  operator- (void) const
    {
        Vec<D,T> vout;
        for(int i=0; i < D; i++)
        {
            vout[i] = -v[i];
        }
        return  vout;
    }
    Vec<D,T>& vectorize(const T& s)
    {
        for(int i=0; i < D;  i++)
        {
            v[i] = s;
        }
        return *this;
    }

    void print() const
    {
        for(size_t i=0; i < D; i++)
        {
            if(fabs((double)v[i]) < TOLERANCE)
            {
                std::cout<<"0 ";
            }
            else
            {
                std::cout<<v[i]<<" ";
            }
        }
        std::cout<<std::endl;
    }
    // if exists an element at index i less than x[i],then return the first index i; else return -1;
    int lessIndex(const Vec<D,T>& x) const
    {
        for(size_t i=0; i < D; i++)
        {
            if(v[i] < x[i] - TOLERANCE)
            {
                return (int)i;
            }
        }
        return -1;
    }
    // if exists an element at index i greater than x[i],then return the first index i; else return -1;
    int greaterIndex(const Vec<D,T>& x) const
    {
        for(size_t i=0; i < D; i++)
        {
            if(v[i] > x[i] + TOLERANCE)
            {
                return (int)i;
            }

        }
        return -1;
    }

    /**
     * @brief      向量=输入向量，向量每个分位最大值组成的向量
     *
     * @param[in]  x     输入向量
     *
     * @return     每个分位取最大值的向量
     */
    Vec<D,T> &max(const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            if(x[i] > v[i])
            {
                v[i] = x[i];
            }
        }
        return *this;
    }

    /**
     * @brief      向量=输入向量，向量每个分位最小值组成的向量
     *
     * @param[in]  x     输入向量
     *
     * @return     每个分位取最小值的向量
     */
    Vec<D,T> &min(const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            if(x[i] < v[i])
            {
                v[i] = x[i];
            }
        }
        return *this;
    }
    Vec<D,T> &operator = (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] = x[i];
        }
        return *this;
    }

    Vec<D,T> &operator += (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] += x[i];
        }
        return *this;
    }

    Vec<D,T> &operator -= (const Vec<D,T> &x)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] -= x[i];
        }
        return *this;
    }

    Vec<D,T> &operator *= (const T &val)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] *= val;
        }
        return *this;
    }
    Vec<D,T> &operator /= (const T &val)
    {
        for(size_t i=0; i < D; i++)
        {
            v[i] /= val;
        }
        return *this;
    }
    T length2() const
    {
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += (v[i] * v[i]);
        }
        return ans;
    }


    T length() const
    {
        using namespace std;
        return sqrt(length2());
    }

    /**
     * @brief      与输入向量互换数值
     *
     * @param      x     输入向量
     */
    void swap(Vec<D,T> &x)
    {
        using namespace std;
        for(size_t i=0; i < D; i++) swap(v[i],x[i]);
    }

    T sum() const
    {
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += v[i];
        }
        return ans;
    }

    /**
     * @brief      向量的1-范数
     *
     * @return     向量各维绝对值之和
     */
    T abssum() const
    {
        using namespace std;
        T ans = (T)0;
        for(size_t i=0; i < D; i++)
        {
            ans += fabs(v[i]);
        }
        return ans;
    }

    /**
     * @brief      向量各维平均值
     *
     * @return     各维平均值
     */
    T avg() const
    {
        return sum() / D;
    }

    /**
     * @brief      向量各维绝对值均值
     *
     * @return     各维绝对值均值
     */
    T absavg() const
    {
        return abssum() / D;
    }

    /**
     * @brief      向量中最大的项
     *
     * @return     最大的项
     */
    T maxvalue() const
    {
        T ans = v[0];
        for(size_t i=1; i < D; i++)
        {
            if(v[i] > ans)
            {
                ans = v[i];
            }
        }
        return ans;
    }

    /**
     * @brief      向量中最小的项
     *
     * @return     最小的项
     */
    T minvalue() const
    {
        T ans = v[0];
        for(size_t i=1; i < D; i++)
        {
            if(v[i] < ans)
            {
                ans = v[i];
            }
        }
        return ans;
    }

    /**
     * @brief      向量最大项所在维度
     *
     * @return     最大项的维度
     */
    size_t maxaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(v[i] > v[id])
            {
                id = i;
            }
        }
        return id;
    }

    /**
     * @brief      向量最小项所在维度
     *
     * @return     最小项的维度
     */
    size_t minaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(v[i] < v[id])
            {
                id = i;
            }
        }
        return id;
    }

    /**
     * @brief      向量绝对值最大的项的绝对值
     *
     * @return     绝对值最大项的绝对值
     */
    T maxabsvalue() const
    {
        T ans = fabs(v[0]);
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) > ans)
            {
                ans = fabs(v[i]);
            }
        }
        return ans;
    }
    /**
     * @brief      向量绝对值最小的项的绝对值
     *
     * @return     绝对值最小项的绝对值
     */
    T minabsvalue() const
    {
        T ans = fabs(v[0]);
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) <ans)
            {
                ans = fabs(v[i]);
            }
        }
        return ans;
    }
    /**
     * @brief      向量绝对值最大的项的维度
     *
     * @return     绝对值最大项的维度
     */
    size_t maxabsaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) > fabs(v[id]))
            {
                id = i;
            }
        }
        return id;
    }
    /**
     * @brief      向量绝对值最小的项的维度
     *
     * @return     绝对值最小项的维度
     */
    size_t minabsaxis() const
    {
        size_t id = 0;
        for(size_t i=1; i < D; i++)
        {
            if(fabs(v[i]) < fabs(v[id]))
            {
                id = i;
            }
        }
        return id;
    }

    /**
     * @brief      func作用在向量上
     *
     * @param[in]  func  函数，返回类型T
     *
     * @return     func作用在向量每一位上产生的新向量
     */
    Vec<D,T> apply(T func(T)) const
    {
        Vec<D,T> result;
        for(size_t i=0; i < D; i++)
        {
            result[i] = func(v[i]);
        }
        return result;
    }
    Vec<D,T> apply(T func(const T&)) const
    {
        Vec<D,T> result;
        for(size_t i=0; i < D; i++)
        {
            result[i] = func(v[i]);
        }
        return result;
    }

    /**
     * @brief      计算点到射线的距离
     *
     * @param[in]  rayPoint      射线上的起 点
     * @param[in]  rayDirection  射线方向
     * @param[in]  point         目标点
     *
     * @return     点到该射线的距离,点在直线上的投影不在射线上时返回0
     * 
     */
    static T getDistanceFromRayToPoint(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                       const Vec<D,T>& point)
    {
        Vec<D,T> raydir = rayDirection;
        raydir.normalize();
        T tmp = (point - rayPoint) * raydir;
        T len2 = (point - rayPoint).length2() - tmp * tmp;
        if(len2 < (T)0) return (T)0;
        return (T)sqrt((len2));
    }

    /**
     * @brief      计算点在射线上的投影
     *
     * @param[in]  point     目标点
     * @param[in]  rayPoint  射线起点
     * @param[in]  rayDir    射线方向向量
     *
     * @return     目标点在射线上的投影
     */
    static Vec<D,T> getClosestPointFromPointToRay(const Vec<D,T>& point, const Vec<D,T>& rayPoint,const  Vec<D,T>& rayDir)
    {
        Vec<D,T> raydir = rayDir;
        raydir.normalize();
        T tmp = (point - rayPoint) * raydir;
        return (rayPoint + raydir * tmp);



    }

    /**
     * @brief      射线与平面是否交
     *
     * @param[in]  rayPoint      射线起点
     * @param[in]  rayDirection  射线方向
     * @param[in]  planePoint    平面上一点
     * @param[in]  planeNormal   平面法向量
     * @param      intersect     交点，如果存在
     *
     * @return     射线与平面是否相交，交点记录在intersect中
     */
    static bool getIntersectionRayToPlane(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                          const Vec<D,T>& planePoint, const Vec<D,T>& planeNormal, Vec<D,T>& intersect)
    {
        T len0 = planeNormal.length();
        if(fabs(len0) < TOLERANCE)
        {
            return false;
        }
        T len1 = rayDirection.length();
        if(fabs(len1) < TOLERANCE)
        {
            return false;
        }
        T t0 = rayDirection * planeNormal;
        T t1 = (rayPoint - planePoint) * planeNormal;
        if(fabs(t0 / (len0 * len1)) < TOLERANCE)
        {
            if(fabs(t1 / len0) > TOLERANCE)
            {
                return false;
            }
            else
            {
                intersect = rayPoint;
                return true;
            }
        }
        T k = -t1 / t0;
        intersect = rayPoint + k * rayDirection;
        return true;
    }

    /**
     * @brief      射线与三角形的交
     *
     * @param[in]  rayPoint      射线起点
     * @param[in]  rayDirection  射线方向
     * @param[in]  triv0         The triv 0
     * @param[in]  triv1         The triv 1
     * @param[in]  triv2         The triv 2
     * @param      intersect     交点 如果存在
     *
     * @return     射线与三角形是否相交
     * 先求射线与三角形平面的交，在判断交点是否在三角形中
     */
    static bool getIntersectionRayToTriangle(const Vec<D,T>& rayPoint, const Vec<D,T>& rayDirection,
                                             const Vec<D,T>& triv0,const Vec<D,T>& triv1, const Vec<D,T>& triv2, Vec<D,T>& intersect)
    {
        Vec<D,T> normal = (triv1 - triv0) ^ (triv2 - triv0);
        normal.normalize();
        if(!getIntersectionRayToPlane(rayPoint,rayDirection,triv0,normal,intersect))
        {
            return false;
        }
        return checkPointInTriangle(intersect,triv0,triv1,triv2);
    }

    /**
     * @brief      检查两条空间线段是否相交
     *
     * @param[in]  s0         起点1
     * @param[in]  s1         终点1
     * @param[in]  t0         起点2
     * @param[in]  t1         终点2
     * @param      intersect  交点
     *
     * @return     是否相交
     * 先检查2线段是否在一平面上，然后检查他们是否相交并返回他们的交点
     */
    static bool checkSegmentsIntersection(const Vec<D,T>& s0, const Vec<D,T>& s1,
                                          const Vec<D,T>& t0, const Vec<D,T>& t1,
                                          Vec<D,T>& intersect)
    {
        // 1: check if the two segments are on the  same plane
        Vec<D,T> normal = (s1 - s0) ^ (t1 - t0);
        normal.normalize();
        if(fabs((t1  -  s0).normalize() * normal) > TOLERANCE)
        {
            return false;
        }
        // 2: check if segment t across segment s
        Vec<D,T> sn = (s1 - s0) ^ normal;
        sn.normalize();
        T ts0 = (t0 - s0) * sn;
        T ts1 = (t1 - s0) * sn;
        if(ts0 * ts1 > -TOLERANCE)
        {
            return false;
        }
        // 3: check if segment s across segment t
        Vec<D,T> tn = (t1 - t0) ^ normal;
        tn.normalize();
        T st0 = (s0 - t0) * tn;
        T st1 = (s1 - t0) * tn;
        if(st0 * st1 > -TOLERANCE)
        {
            return false;
        }
        // 4: get intersect
        //intersect = t0 + (t1 - t0) *  (fabs(ts0) / (t1 - t0).length());
        intersect = t0 + (t1 - t0) *  (fabs(ts0) / fabs((t1 - t0)*sn));
        return true;
    }

    /**
     * @brief      checkPointInTriangle If point near the boundary of triangle,
     *             it will be checked to be inside the triangle; More accuracy
     *             should be done in further computing by user;
     *
     * @param[in]  point  目标点
     * @param[in]  triv0  The triv 0
     * @param[in]  triv1  The triv 1
     * @param[in]  triv2  The triv 2
     *
     * @return     目标点是否在三角形中
     *       
     */
    static int checkPointInTriangle(const Vec<D,T>& point,
                                     const Vec<D,T>& triv0,const Vec<D,T>& triv1, const Vec<D,T>& triv2)
    {
        // check if the point on the triv0,triv1,triv2
        if((triv0 - point).length() < TOLERANCE)
        {
            return true;
        }
        if((triv1 - point).length() < TOLERANCE)
        {
            return true;
        }
        if((triv2 - point).length() < TOLERANCE)
        {
            return true;
        }
        // check if the point on the plane
        Vec<D,T> normal = (triv1 - triv0) ^ (triv2 - triv0);
        normal.normalize();
        Vec<D,T> tv0 = point - triv0;
        tv0.normalize();
        if(fabs(tv0 * normal) > TOLERANCE)
        {
            return false;
        }
        // check if the point inside the triangle
        Vec<D,T> n = normal ^ (triv1 - triv0);
        n.normalize();
        T side0 = (triv2 - triv0).normalize() * n;
        T side1 = (point - triv0).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        n = normal ^ (triv2 - triv1);
        n.normalize();
        side0 = (triv0 - triv1).normalize() * n;
        side1 = (point - triv1).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        n = normal ^ (triv0 - triv2);
        n.normalize();
        side0 = (triv1 - triv2).normalize() * n;
        side1 = (point - triv2).normalize() * n;
        if(side0 * side1 < -TOLERANCE)
        {
            return false;
        }
        return true;

    }
private:
    T v[D];
};

/**
 * @brief      向量内积
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     D     维数
 * @tparam     T     每位数据类型
 *
 * @return     内积
 */
template <size_t D, class T>
static inline const T dot(const Vec<D,T> &v1, const Vec<D,T>& v2)
{
    T ans = (T)0;
    for(size_t i=0; i < D; i++)
    {
        ans += (v1[i] * v2[i]);
    }
    return ans;
}

/**
 * @brief      三维向量外积
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     T     每位数据类型
 *
 * @return     外积
 */
template <class T>
static inline const Vec<3,T> cross(const Vec<3,T> &v1, const Vec<3,T>& v2)
{
    return Vec<3,T>(v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]);
}
template <size_t D, class T>
static inline const Vec<D,T> operator + (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

template <size_t D, class T>
static inline const Vec<D,T> operator - (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}
/**
 * @brief      向量内积
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     D     维数
 * @tparam     T     每位数据类型
 *
 * @return     内积
 */
template <size_t D, class T>
static inline const T operator * (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    T ans = (T)0;
    for(size_t i=0; i < D; i++)
    {
        ans += (v1[i] * v2[i]);
    }
    return ans;
}


/**
 * @brief      向量乘数
 *
 * @param[in]  val   The value
 * @param[in]  v1    The v 1
 *
 * @tparam     D     维数
 * @tparam     T     向量数据类型
 * @tparam     T2    乘数数据类型
 *
 * @return     向量乘数结果
 */
template <size_t D, class T,class T2>
static inline const Vec<D,T> operator * (const Vec<D,T> &v1, const T2 &val)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = (v1[i] * val);
    }
    return result;
}
/**
 * @brief      向量除数
 *
 * @param[in]  val   The value
 * @param[in]  v1    The v 1
 *
 * @tparam     D     维数
 * @tparam     T     向量数据类型
 * @tparam     T2    除数数据类型
 *
 * @return     向量除数结果
 */
template <size_t D, class T,class T2>
static inline const Vec<D,T> operator / (const Vec<D,T> &v1, const T2 &val)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = v1[i] / (T)val;
    }
    return result;
}

/**
 * @brief      向量乘数
 *
 * @param[in]  val   The value
 * @param[in]  v1    The v 1
 *
 * @tparam     D     维数
 * @tparam     T     向量数据类型
 * @tparam     T2    乘数数据类型
 *
 * @return     向量乘数结果
 */
template <size_t D, class T, class T2>
static inline const Vec<D,T> operator * ( const T2 &val,const Vec<D,T> &v1)
{
    Vec<D,T> result;
    for(size_t i=0; i < D; i++)
    {
        result[i] = (v1[i] * (T)val);
    }
    return result;
}


/**
 * @brief      三维向量外积
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     T     每位数据类型
 *
 * @return     外积
 */
template <class T>
static inline const Vec<3,T> operator ^ (const Vec<3,T> &v1, const Vec<3,T> &v2)
{

    return Vec<3,T>(v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]);
}

template <size_t D, class T>
static inline  bool operator == (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    return ((v1-v2).length() < TOLERANCE);
}

template <size_t D, class T>
static inline  bool operator != (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    return !(v1 == v2);
}

/**
 * @brief      判断向量1是否每位都小于向量2
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     D     维数
 * @tparam     T     数据类型
 *
 * @return     判断结果
 */
template <size_t D, class T>
static inline  bool operator < (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    for(size_t i=0; i < D; i++)
    {
        if(v1[i] > v2[i] + TOLERANCE) return false;
        else if(v1[i] < v2[i] - TOLERANCE) return true;
    }
    return false;
}
/**
 * @brief      判断向量1是否每位都大于向量2
 *
 * @param[in]  v1    向量1
 * @param[in]  v2    向量2
 *
 * @tparam     D     维数
 * @tparam     T     数据类型
 *
 * @return     判断结果
 */
template <size_t D, class T>
static inline  bool operator > (const Vec<D,T> &v1, const Vec<D,T> &v2)
{
    for(size_t i=0; i < D; i++)
    {
        if(v1[i] < v2[i] - TOLERANCE) return false;
        else if(v1[i] > v2[i] + TOLERANCE) return true;
    }
    return false;
}

template <size_t D, class T>
static inline  bool operator <= (const Vec<D,T> &v1, const Vec<D,T> &v2)
{

    return !(v1 > v2);
}
template <size_t D, class T>
static inline  bool operator >= (const Vec<D,T> &v1, const Vec<D,T> &v2)
{

    return !(v1 < v2);
}

/**
 * @brief      4维向量转256RGB
 *
 * @param[in]  v     目标向量
 *
 * @tparam     T     数据类型
 *
 * @return     输出RGB向量
 * 把输入向量截取进0-1范围，都乘以255输出
 */
template <class T>
static inline Vec<4,T> convertToRGBA256(const Vec<4,T> &v)
{
    Vec<4,T> v1 = v;
    for(size_t i=0; i < 4; i++)
    {
        if(v1[i] > 1)
        {
            v1[i] = 1;
        }
        if(v1[i] < 0)
        {
            v1[i] = 0;
        }
    }
    for(size_t i=0; i < 4; i++)
    {
        v1[i] = v1[i] * (T)255;
    }
    return v1;
}


/**
 * @brief      256RGB转化4维向量
 *
 * @param[in]  v     RGB向量
 *
 * @tparam     T     数据类型
 *
 * @return     输出向量
 */
template <class T>
static inline Vec<4,T> convertFromRGBA256(const Vec<4,T> &v)
{
    Vec<4,T> v1 = v;
    for(size_t i=0; i < 4; i++)
    {
        if(v1[i] > 255)
        {
            v1[i] = 255;
        }
        if(v1[i] < 0)
        {
            v1[i] = 0;
        }
    }
    for(size_t i=0; i < 4; i++)
    {
        v1[i] = v1[i] / (T)255;
    }
    return v1;
}

template<class T>
Vec<3, T> &normalize(Vec<3, T> &v)
{
    return v.normalize();
}

template<class T>
T norm(const Vec<3, T> &v)
{
    return v.norm();
}

template <size_t D, class T1, class T2>
Vec<D, T1> &vectorize(Vec<D, T1> &v, T2 const &s)
{
    for(size_t i = 0; i < D; i++)
        v[i] = s;
    return v;
}

}


#endif // VEC_H
