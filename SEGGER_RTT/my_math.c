#include "my_math.h"

//数学函数	

/* 
函数功能:向上取整
参数:x - 被操作数  
返回值:x向上取整值
*/
int Round(float x) { //向上取整函数
		int a;
		a = (int)(x*10+5)/10.0;  
		return a;
}
/* 
函数功能:向下取整
参数:x - 被操作数  
返回值:x向下取整值
*/
int Floor(float x) { // 向下取整函数
    int a;
    a = (int)(x * 10) / 10;  
    return a;
}
/* 
函数功能:四舍五入
参数:num - 被操作数    
返回值:num四舍五入值
*/
int RoundToNearest(float num) {//四舍五入
    // 获取小数部分
    float decimalPart = num - (int)num;
    
    // 如果小数部分大于等于0.5，向上取整；否则，向下取整
    if (decimalPart >= 0.5) {
        return (int)num + 1;
    } else {
        return (int)num;
    }
}
/* 
函数功能:三次样条插值
参数:x,y - 坐标点
	n - 插值点
返回值:n对应y坐标
*/
float cubic_interpolation(float x1,float y1,float x2,float y2,float n){//三次样条插值函数，x1,y1.x2.y2两个坐标点，
	float a,b,c,d,p;
	float pos;
	a=-2*(y1-y2)/((x1-x2)*(x1-x2)*(x1-x2));
	b=-3*a*(x1+x2)/2;
	c=3*a*x1*x2;
	d=y1-a*x1*x1*x1-b*x1*x1-c*x1;	
	p=a*n*n*n+b*n*n+c*n+d;
	pos = p;
	// pos=Round(p);
	if(n<0)
	{return 0;}	
	else
	{return pos;}
}
/* 
函数功能:线性插值
参数:x,y - 坐标点
	n - 插值点
返回值:n对应y坐标
*/
float Linear_interpolation(float x1,float y1,float x2,float y2,float Phase)//线性插值
{
	float k,p;
	int pos;
	
	k = (y2-y1)/(x2-x1);
	p = y1+k*(Phase-x1);
	return p;
}
/* 
函数功能:线性插值
参数:x,y - 坐标点
	n - 插值点
返回值:斜率
*/
float Linear_Velocity_interpolation(float x1,float y1,float x2,float y2)//线性插值
{
	float k;
	k = (y2-y1)/(x2-x1);
	return k;
}
/* 
函数功能:将鲍登线位移转换为电机转动角度
参数:Displacement - 鲍登线位移，单位mm
返回值:电机转动角度
*/
float DisplacementToDegree(float Displacement)
{
	int Diameter = 62;//绕线轮直径，单位mm
	return (Displacement)*360/(Pi*Diameter);//鲍登线位移转换为电机转动角度
}
/* 
函数功能:将鲍登线位移转换为电机转动角度
参数:Degree - 电机转动角度
返回值:鲍登线位移，单位mm
*/
float DegreeToDisplacement(float Degree)
{
	int Diameter = 62;//绕线轮直径
	return (Degree)*Pi*Diameter/(360);//电机转动角度转位移
}

/* 
函数功能:根据期望峰值力计算位移
参数:force - 期望力
返回值:鲍登线位移
*/
float ForceToDisplacement(int force)//根据期望峰值力计算位移
{
	float Displacement;
	float Stiffness = 2.4;//刚度实验测得刚度
	Displacement = force/Stiffness;//刚度实验测得函数
	return Displacement;
}

/* 
函数功能:计算绝对值
参数:num - 被操作数
返回值:num绝对值
*/
float absoluteValue(float num) {
    if (num < 0) {
        return -num;
    } else {
        return num;
    }
}