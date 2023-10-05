#include "../include/line_fsc/line_fsc.hpp"

namespace ns_line_fsc {

// set function
void LineFSC::setPerceptionData(sensor_msgs::PointCloud msg) {
    this->perception_data = msg;
    this->get_cloud_flag = true;
}

void LineFSC::setVelocity(float vel) {
    this->velocity = vel;
    this->get_vel_flag = true;
}

// get function
float LineFSC::getDelta() {
    return this->delta;
}

// main function
void LineFSC::runAlgorithm() {
    if(!this->get_cloud_flag || !this->get_vel_flag) { return; }
    
    // 得到左右桩桶分割阈值
    float threshold = 0;
    for(int i = 0; i < this->perception_data.points.size(); i++) { threshold += this->perception_data.points[i].y; }
    threshold /= this->perception_data.points.size();
    
    // 区分左右桩桶
    sensor_msgs::PointCloud points_right, points_left;
    for(int i = 0; i < this->perception_data.points.size(); i++) {
        if(this->perception_data.points[i].y > threshold) {
            points_left.points.push_back(this->perception_data.points[i]);
        } else {
            points_right.points.push_back(this->perception_data.points[i]);
        }
    }

    // 左右桩桶分别进行直线拟合
    LinearFitting(points_left, this->k_left, this->b_left);
    LinearFitting(points_right, this->k_right, this->b_right);
    // 计算中线斜率
    ave(this->k_left, this->b_left, this->k_right, this->b_right, this->k_lane, this->b_lane);
    // 计算中线与坐标原点（即车辆位置）的垂直距离
    distance(this->k_lane, this->b_lane);
    // 计算转向角度
    stanley(this->velocity, this->k_lane, this->dist);
    // 设置flag为true从而使publisher可以发送消息
    run_flag = true;

    // 测试代码
    // std::cout<<"k_left:"<<this->k_left<<" b_left:"<<this->b_left<<std::endl;
    // std::cout<<"k_right:"<<this->k_right<<" b_right:"<<this->b_right<<std::endl;
    // std::cout<<"k_lane:"<<this->k_lane<<" b_lane:"<<this->b_lane<<std::endl;
    // std::cout<<"dist:"<<this->dist<<" angle:"<<this->delta<<std::endl;
}

// 最小二乘法拟合直线
void LineFSC::LinearFitting(sensor_msgs::PointCloud points, float &k, float &b) {
    int count = points.points.size();
	double sumX = 0, sumY = 0, sumXY = 0, sumSqrX = 0;
	for (int i = 0; i < count; i++)
	{
		sumX += points.points[i].x;
		sumY += points.points[i].y;
		sumXY += points.points[i].x * points.points[i].y;
		sumSqrX += points.points[i].x * points.points[i].x;
	}
    k = (count*sumXY - sumX*sumY)/(count*sumSqrX - sumX*sumX);
    b = (sumY*sumSqrX - sumX*sumXY)/(count*sumSqrX - sumX*sumX);
}

// 平分线计算
void LineFSC::ave(float k1, float b1, float k2, float b2, float &k, float &b) {
    if (k1!=k2){
    k=((k1*k2+sqrt(k1*k1*k2*k2+k1*k1+k2*k2+1)-1)/(k1+k2));
    if (std::min(k1,k2)<k & std::max(k1,k2)>k) k=k;   
    else{k=((k1*k2-sqrt(k1*k1*k2*k2+k1*k1+k2*k2+1)-1)/(k1+k2));}

    b=((b2*k1-b1*k2)/(k1-k2)-k*(b2-b1)/(k1-k2));
    }
    else{
        k=k1;
        b=(b1+b2)/2;
    }
}

// 自车坐标原点到轨迹的偏距
void LineFSC::distance(float k, float b, float x, float y) {
    if (this->b_lane > 0) {
        this->dist = abs(k*x-y+b)/sqrt(k*k+1);
    } else {
        this->dist = -abs(k*x-y+b)/sqrt(k*k+1);
    }   
}

// 计算转向角度
void LineFSC::stanley(float vel, float k, float e) {
    if(abs(vel) < this->vel_limit)
    {   
        this->delta = 0;
        return;
    }
    else{
        float theta  = atan(k);
        // Retrieved from Parameter server
        float a = atan(this->k_e * e / (vel + this->k_v));
        this->delta = 180*(a + theta)/3.14;
    } 
}

}