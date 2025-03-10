//
// Created by 26090 on 25-1-17.
//

#include "rv2_trajectory.h"

/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include <math.h>
#include <stdio.h>

#include "rv2_trajectory.h"
#include "rv2_protocal.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间


trajectory_target_s trajectory_target;

float test_bias= 0.3;   //s_bias
float test_t_bias = 100;

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{

    return 0;

}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        // printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
        //     i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{   
    st.bias_time = test_t_bias;
    // 线性预测
    float timeDelay = st.bias_time/1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //计算四块装甲板的位置
    //反馈数据只会反馈最前面识别到的一块装甲板位置，但是我们可以通过这个识别的装甲板位置和r1/r2来推算其他装甲板位置，从而实现瞄准决策！

    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板

    //根据装甲板数量切换目标击打方式

    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用

        //暂时套用击打普通4装甲的代码
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<3; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    } else {

        //普通步兵
        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
            //Todo:*yaw似乎并非实际yaw。但是也可以用，因为这样跟踪效果较好，而且切换较快
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

    //对选择的装甲板进行击打目标计算

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    st.s_bias = test_bias;
    //这里符号给错了
    float temp_pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}

float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
float pitch = 0; //输出控制量 pitch绝对角度 弧度
float yaw = 0;   //输出控制量 yaw绝对角度 弧度

// 从坐标轴正向看向原点，逆时针方向为正

trajectory_target_s *rv2_trajectory_init()
{
    //定义参数
    st.k = 0.092;
    st.bullet_type =  BULLET_17;
    st.current_v = 21;//18
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.xw = 0.0;
    st.yw = 0;
    st.zw = 0;

    st.vxw = 0;
    st.vyw = 0;
    st.vzw = 0;
    st.v_yaw = 0;
    st.tar_yaw = 0;
    st.r1 = 0;
    st.r2 = 0;
    st.dz = 0;

    //以下设置参数
    st.bias_time = 10;
    st.s_bias = 0.07;  //0.2
    st.z_bias = 0.06;   //0.19
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;

    // printf("main pitch:%f° yaw:%f° ", pitch * 180 / PI, yaw * 180 / PI);
    // printf("\npitch:%frad yaw:%frad aim_x:%f aim_y:%f aim_z:%f", pitch, yaw, aim_x, aim_y, aim_z);

    return &trajectory_target;
}

void rv2_trajectory_passin(rv2_recv_protocol_s *param1,float *param2)
{
    st.xw = param1->x;
    st.yw = param1->y;
    st.zw = param1->z;
    st.tar_yaw = param1->yaw;
    st.vxw = param1->vx;
    st.vyw = param1->vy;
    st.vzw = param1->vz;
    st.v_yaw = param1->v_yaw;
    st.r1 = param1->r1;
    st.r2 = param1->r2;
    st.dz = param1->dz;

    st.armor_id=param1->id;
    st.armor_num=param1->armors_num;

    st.current_pitch=param2[1];
    st.current_yaw = param2[0];


}

void rv2_trajectory_calculate()
{

    //预测
    autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);

    //返回值
    trajectory_target.pitch=pitch;
    trajectory_target.yaw=yaw;
    trajectory_target.aim_x=aim_x;
    trajectory_target.aim_y=aim_y;
    trajectory_target.aim_z=aim_z;
}


