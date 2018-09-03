#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

class PinholeCamera
{
public:
    PinholeCamera(double width, double height, double fx, double fy, double cx, double cy,
                    double k1, double k2, double p1, double p2, double k3);
    
    ~PinholeCamera();


    //任何不会修改数据成员(即函数中的变量)的函数都应该声明为const 类型
    //在函数定义头后面加上的 const 表示这个函数是一个“只读函数”，函数不能改变类对象的状态，不能改变对象的成员变量的值
    inline double width() const { return width_; }
    inline double height() const { return height_; }
	inline double fx() const { return fx_; };
	inline double fy() const { return fy_; };
	inline double cx() const { return cx_; };
	inline double cy() const { return cy_; };
	inline double k1() const { return d_[0]; };
	inline double k2() const { return d_[1]; };
	inline double p1() const { return d_[2]; };
	inline double p2() const { return d_[3]; };
	inline double k3() const { return d_[4]; };

private:

    double width_; 
    double height_; 
    double fx_; 
    double fy_; 
    double cx_; 
    double cy_;
    double d_[5];

};
# endif //PINHOLE_CAMERA_H_