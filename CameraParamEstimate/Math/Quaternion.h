#ifndef QUATERNION_H
#define QUATERNION_H
#include <math.h>
#include "HomoMatrix.h"
#include <iostream>

namespace GCL {


/**
 * @brief      四元数体代表三维空间旋转变换
 *
 * @tparam     T     数据类型
 * OPenGL中常用的变换类型
 * 四元数体基本定义和运算：
 * 四元数体 q= x*i +y*j +z*k +w
 * i*i=j*j=k*k=ijk=-1
 * i*j=k j*k=i k*i=j
 * 乘法不可交换
 * 四元数体乘法可代表3维空间过原点的旋转变换
 * 
 */
template<class T = float>
class Quaternion
{
public:

    /**
     * @brief      初始化，4个项都设为0
     */
    Quaternion(){
        mX = mY = mZ = mW = 0;
    }

    /**
     * @brief      从给定xyzw初始化
     *
     * @param[in]  x     x
     * @param[in]  y     y
     * @param[in]  z     z
     * @param[in]  w     w
     */
    Quaternion(const T& x,const T& y, const T& z,const T& w)
    {
        mX = x;  mY = y; mZ = z; mW = w;
    }

    Quaternion(const Vec<4,T>& v)
    {
        mX = v[0];mY = v[1]; mZ = v[2]; mW = v[3];
    }
    /**
     * @brief      从轴以及旋转角度转化为四元数体
     *
     * @param[in]  axis   轴
     * @param[in]  angle  旋转角度
     * 轴（x,y,z） 转angle 化为四元数体所需要的四元数
     * q=((x,y,z)sin(angle/2),cos(angle/2))
     * 证明: 把要变换的点P写为p(P，0)
     * 先证明模长不变 :|qpq-1|=|p|
     * set q-1=q* 定义 S(q)=(q+q*)/2
     * 由：
     * 2S(qpq-1)=2S(qpq*)=qpq*+qp*q*=q2S(p)q*=2S(p)
     * 推出：|qpq-1|=|p|（把S（p）平方即可得）
     * 然后让|p|, |u(x,y,z)|都=1, k 是p绕u旋转angle/2所得向量
     * 那么又向量内积外积定义
     * u sin(a/2)=p cross k， cos(a/2)=p dot k
     * 得出
     * q=kp*
     * 下面检验 w=qpq* 就是p旋转angle所得向量
     *  wk*=(qvq*)k*=qppk*k*=q 
     *  所以w是k旋转angle/2得到的
     *  
     *  
     * 证毕
     * 
     */
    Quaternion(const Vec<3,T>& axis, const T& angle)
    {

        Vec<3,T> taxis(axis);
        if(taxis.Normalize() < std::numeric_limits<T>::min())
        {
            mX = mY = mZ = mW = 0;
            return;
        }
        mX = taxis[0] * sin(angle / 2);
        mY = taxis[1] * sin(angle / 2);
        mZ = taxis[2] * sin(angle / 2);
        mW = cos(angle / 2);
    }



    void print() const
    {
        std::cout<<mX<<" "<<mY<<" "<<mZ<<" "<<mW<<std::endl;
    }
    /**
     * @brief      四元数体模长平方
     *
     * @return     x^2+y^2+z^2+w^2
     */
    T length2() const
    {
        return mX * mX + mY * mY + mZ * mZ + mW * mW;
    }
    T length() const
    {
      return sqrt(length2());
    }
    /**
     * @brief      四元数体的共轭数
     *
     * @return     q=(v,w),q*=(-v,w);
     */
    Quaternion<T> conjugate() const
    {
        Quaternion<T> q(-mX,-mY,-mZ,mW);
        return q;
    }
    /**
     * @brief      单位化
     *
     * @return     四元数体除以模长
     */
    T Normalize()
    {
        T len = length();
        if(len <= (T)std::numeric_limits<T>::min())
        {
            return len;
        }
        mX /= len;
        mY /= len;
        mZ /= len;
        mW /= len;
        return (T)1;
    }
    /**
     * @brief      返回轴
     *
     * @return     Vec(x,y,z)
     */
    Vec<3,T> getAxis() const
    {
        return Vec<3,T>(mX,mY,mZ);
    }
    /**
     * @brief      返回旋转角
     *
     * @return     w
     */
    T getAngle() const
    {
        return acos(mW) * 2;
    }
    /**
     * @brief      从4阶Homo变换矩阵转化为四元数体
     *
     * @param[in]  m     输入4阶矩阵
     * 由
     *      1−2(y^2+z^2)    2(xy+zw)        2(xz−yw)
     * R(q)=2(xy−zw)        1−2(x^2+z^2)    2(yz+xw)
     *      2(xz+yw)        2(yz−xw)        1−2(x^2+y^2)
     * 可分别求 x+y y+z z+x
     * 从而求出x,y,z,w
     * 
     */
    void convertFromMatrix(const HomoMatrix<4,T>& m)
    {
        T s;
        T tq[4];
        int i,j;

        // Use tq to store the largest trace
        tq[0] = 1 + m.getValue(0,0)+m.getValue(1,1)+m.getValue(2,2);
        tq[1] = 1 + m.getValue(0,0)-m.getValue(1,1)-m.getValue(2,2);
        tq[2] = 1 - m.getValue(0,0)+m.getValue(1,1)-m.getValue(2,2);
        tq[3] = 1 - m.getValue(0,0)-m.getValue(1,1)+m.getValue(2,2);

        // Find the maximum (could also use stacked if's later)
        j = 0;
        for(i=1;i<4;i++) j = (tq[i]>tq[j])? i : j;

        // check the diagonal
        if (j==0)
        {
            /* perform instant calculation */
            mW = tq[0];
            mX = m.getValue(1,2)-m.getValue(2,1);
            mY = m.getValue(2,0)-m.getValue(0,2);
            mZ = m.getValue(0,1)-m.getValue(1,0);
        }
        else if (j==1)
        {
            mW = m.getValue(1,2)-m.getValue(2,1);
            mX = tq[1];
            mY = m.getValue(0,1)+m.getValue(1,0);
            mZ = m.getValue(2,0)+m.getValue(0,2);
        }
        else if (j==2)
        {
            mW = m.getValue(2,0)-m.getValue(0,2);
            mX = m.getValue(0,1)+m.getValue(1,0);
            mY = tq[2];
            mZ = m.getValue(1,2)+m.getValue(2,1);
        }
        else /* if (j==3) */
        {
            mW = m.getValue(0,1)-m.getValue(1,0);
            mX = m.getValue(2,0)+m.getValue(0,2);
            mY = m.getValue(1,2)+m.getValue(2,1);
            mZ = tq[3];
        }
        s = sqrt(0.25/tq[j]);
        mW *= s;
        mX *= -s;
        mY *= -s;
        mZ *= -s;
    }

    /**
     * @brief      转换为4阶OpenGL矩阵
     *
     * @return     4阶OpenGL矩阵
     * q=xi+yj+zk+w 作用在p(x1,x2,x3)上结果是
     * 展开qpq* 观察每项中x1，x2，x3中系数即得
     *      1−2(y^2+z^2)    2(xy+zw)        2(xz−yw)
     * R(q)=2(xy−zw)        1−2(x^2+z^2)    2(yz+xw)
     *      2(xz+yw)        2(yz−xw)        1−2(x^2+y^2)
     * 
     */
    HomoMatrix<4,T> convertToMatrix() const
    {
        HomoMatrix<4,T> mat;
        T s, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;
        T sum = ((mX * mX) + (mY * mY) +
                 (mZ * mZ) + (mW * mW));
        if(fabs(sum) < std::numeric_limits<T>::min())
        {
            return mat;
        }
        s = 2.0 / sum;

        xs = mX * s;
        ys = mY * s;
        zs = mZ * s;

        wx = mW * xs;
        wy = mW * ys;
        wz = mW * zs;

        xx = mX * xs;
        xy = mX * ys;
        xz = mX * zs;

        yy = mY * ys;
        yz = mY * zs;

        zz = mZ * zs;



        /*!
         Fill in matrix

       */
        mat.setValue(0,0,1.0 - (yy + zz));
        mat.setValue(0,1, xy - wz);
        mat.setValue(0,2 , xz + wy);

        mat.setValue(1,0, xy + wz);
        mat.setValue(1,1, 1.0 - (xx + zz));
        mat.setValue(1,2,yz - wx);

        mat.setValue(2,0,  xz - wy);
        mat.setValue(2,1,yz + wx);
        mat.setValue(2,2,1.0 - (xx + yy));

        for(int i=0; i < 3; i++)
        {
            mat.setValue(i,3,0);
            mat.setValue(3,i,0);
        }
        mat.setValue(3,3,1);
        return mat;

    }
    const T& x() const {return mX;}
    const T& y() const {return mY;}
    const T& z() const {return mZ;}
    const T& w() const {return mW;}
    T& x() {return mX;}
    T& y() {return mY;}
    T& z() {return mZ;}
    T& w() {return mW;}

    /**
     * @brief      从向量旋转结果反推变换四元数体
     *
     * @param[in]  tv0   初始向量
     * @param[in]  tv1   结果向量
     *
     * @return     旋转四元数体
     * 外积找出旋转轴，内积求出旋转角度，调用前面的设定求出所要四元数体
     */
    static  Quaternion<T> quatFromVectorTransform(const Vec<3,T>& tv0, const Vec<3,T>& tv1)
    {
        Vec<3,T> v0(tv0);
        Vec<3,T> v1(tv1);
        v0.normalize();
        v1.normalize();
        Vec<3,T> axis = v0 ^ v1;
        if(axis.Normalize() < std::numeric_limits<T>::min())
        {
            axis = v0 ^ Vec<3,T>(1,0,0);
            if(axis.Normalize() < std::numeric_limits<T>::min())
            {
                axis = v0 ^ Vec<3,T>(0,1,0);
            }
        }
        T angle = acos(v0 * v1);
        Quaternion<T> ans(axis,angle);
        return ans;
    }
private:
    T mX,mY,mZ,mW;
};
///四元数体基本运算 加减乘
template <class T>
static inline const Quaternion<T> operator + (const Quaternion<T>  &v1, const Quaternion<T>  &v2)
{
    Quaternion<T>  result;
    result.x() = v1.x() + v2.x();
    result.y() = v1.y() + v2.y();
    result.z() = v1.z() + v2.z();
    result.w() = v1.w() + v2.w();
    return result;
}

template <class T>
static inline const Quaternion<T> operator - (const Quaternion<T>  &v1, const Quaternion<T>  &v2)
{
    Quaternion<T>  result;
    result.x() = v1.x() - v2.x();
    result.y() = v1.y() - v2.y();
    result.z() = v1.z() - v2.z();
    result.w() = v1.w() - v2.w();
    return result;
}

template <class T>
static inline const Quaternion<T> operator * (const Quaternion<T>  &left, const Quaternion<T>  &right)
{
    Quaternion<T> ans;
    T d1, d2, d3, d4;

    d1 =  left.w() * right.w();
    d2 = -left.x() * right.x();
    d3 = -left.y() * right.y();
    d4 = -left.z() * right.z();
    ans.w() = d1+ d2+ d3+ d4;

    d1 =  left.w() * right.x();
    d2 =  right.w() * left.x();
    d3 =  left.y() * right.z();
    d4 = -left.z() * right.y();
    ans.x() =  d1+ d2+ d3+ d4;

    d1 =  left.w() * right.y();
    d2 =  right.w() * left.y();
    d3 =  left.z() * right.x();
    d4 = -left.x() * right.z();
    ans.y() =  d1+ d2+ d3+ d4;

    d1 =  left.w() * right.z();
    d2 =  right.w() * left.z();
    d3 =  left.x() * right.y();
    d4 = -left.y() * right.x();
    ans.z() =  d1+ d2+ d3+ d4;
    return ans;
}
/**
 * @brief      四元数体对向量进行旋转变换
 *
 * @param[in]  quat0  变换四元数体
 * @param[in]  v      要操作的向量
 *
 * @tparam     T      数据类型
 *
 * @return     旋转后向量
 * 把向量P设为4元p(P,0)
 * qpq*即为旋转后向量
 */
template <class T>
static inline const Vec<3,T> operator * (const Quaternion<T>  &quat0, const Vec<3,T>  &v)
{
    Quaternion<T> quat1(v[0],v[1],v[2],0);
    Quaternion<T> ans = quat0 * quat1;
    ans = ans * quat0.conjugate();
    Vec<3,T> ansV(ans.x(),ans.y(),ans.z());
    return ansV;
}

}
#endif // QUATERNION_H
