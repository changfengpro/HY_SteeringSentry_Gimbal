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

struct SolveTrajectoryParams st_l;
struct tar_pos tar_position_l[4]; //最多只有四块装甲板
float t_l = 0.5f; // 飞行时间


trajectory_target_s trajectory_target_l;

float test_l_bias= 0.3;   //s_bias
float test_l_t_bias = 100;

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel_L(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t_l = (float)((exp(st_l.k * s) - 1) / (st_l.k * v * cos(angle)));
    if(t_l < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t_l = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t_l - GRAVITY * t_l * t_l / 2);
    printf("model %f %f\n", t_l, z);
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
float completeAirResist_lanceModel_L(float s, float v, float angle)
{

    return 0;

}



/*
@brief pitch_l轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch_l:rad
*/
float pitch_lTrajectoryCompensation_L(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch_l;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch_l = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel_L(s, v, angle_pitch_l);
        if(z_actual == 0)
        {
            angle_pitch_l = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        // printf("iteration num %d: angle_pitch_l %f, temp target z:%f, err of z:%f, s:%f\n",
        //     i + 1, angle_pitch_l * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch_l;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch_l:rad  传出pitch_l
@param yaw_l:rad    传出yaw_l
@param aim_x_l:传出aim_x_l  打击目标的x
@param aim_y_l:传出aim_y_l  打击目标的y
@param aim_z_l:传出aim_z_l  打击目标的z
*/
void autoSolveTrajectory_L(float *pitch_l, float *yaw_l, float *aim_x_l, float *aim_y_l, float *aim_z_l)
{   
    st_l.bias_time = test_l_t_bias;
    // 线性预测
    float timeDelay = st_l.bias_time/1000.0 + t_l;
    st_l.tar_yaw += st_l.v_yaw * timeDelay;

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
    if (st_l.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw_l = st_l.tar_yaw + i * PI;
            float r = st_l.r1;
            tar_position_l[i].x = st_l.xw - r*cos(tmp_yaw_l);
            tar_position_l[i].y = st_l.yw - r*sin(tmp_yaw_l);
            tar_position_l[i].z = st_l.zw;
            tar_position_l[i].yaw = tmp_yaw_l;
        }

        float yaw_l_diff_min = fabsf(*yaw_l - tar_position_l[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_l_diff = fabsf(*yaw_l - tar_position_l[1].yaw);
        if (temp_yaw_l_diff < yaw_l_diff_min)
        {
            yaw_l_diff_min = temp_yaw_l_diff;
            idx = 1;
        }


    } else if (st_l.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw_l = st_l.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st_l.r1 + st_l.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position_l[i].x = st_l.xw - r*cos(tmp_yaw_l);
            tar_position_l[i].y = st_l.yw - r*sin(tmp_yaw_l);
            tar_position_l[i].z = st_l.zw;
            tar_position_l[i].yaw = tmp_yaw_l;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用

        //暂时套用击打普通4装甲的代码
        float yaw_l_diff_min = fabsf(*yaw_l - tar_position_l[0].yaw);
        for (i = 1; i<3; i++) {
            float temp_yaw_l_diff = fabsf(*yaw_l - tar_position_l[i].yaw);
            if (temp_yaw_l_diff < yaw_l_diff_min)
            {
                yaw_l_diff_min = temp_yaw_l_diff;
                idx = i;
            }
        }

    } else {

        //普通步兵
        for (i = 0; i<4; i++) {
            float tmp_yaw_l = st_l.tar_yaw + i * PI/2.0;
            float r = use_1 ? st_l.r1 : st_l.r2;
            tar_position_l[i].x = st_l.xw - r*cos(tmp_yaw_l);
            tar_position_l[i].y = st_l.yw - r*sin(tmp_yaw_l);
            tar_position_l[i].z = use_1 ? st_l.zw : st_l.zw + st_l.dz;
            tar_position_l[i].yaw = tmp_yaw_l;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw_l最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position_l[0].x * tar_position_l[0].x + tar_position_l[0].y * tar_position_l[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position_l[i].x * tar_position_l[0].x + tar_position_l[i].y * tar_position_l[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw_l最小的那个装甲板
            //Todo:*yaw_l似乎并非实际yaw_l。但是也可以用，因为这样跟踪效果较好，而且切换较快
        float yaw_l_diff_min = fabsf(*yaw_l - tar_position_l[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_l_diff = fabsf(*yaw_l - tar_position_l[i].yaw);
            if (temp_yaw_l_diff < yaw_l_diff_min)
            {
                yaw_l_diff_min = temp_yaw_l_diff;
                idx = i;
            }
        }

    }

    //对选择的装甲板进行击打目标计算

    *aim_z_l = tar_position_l[idx].z + st_l.vzw * timeDelay;
    *aim_x_l = tar_position_l[idx].x + st_l.vxw * timeDelay;
    *aim_y_l = tar_position_l[idx].y + st_l.vyw * timeDelay;
    st_l.s_bias = test_l_bias;
    //这里符号给错了
    float temp_pitch_l = -pitch_lTrajectoryCompensation_L(sqrt((*aim_x_l) * (*aim_x_l) + (*aim_y_l) * (*aim_y_l)) - st_l.s_bias,
            *aim_z_l + st_l.z_bias, st_l.current_v);
    if(temp_pitch_l)
        *pitch_l = temp_pitch_l;
    if(*aim_x_l || *aim_y_l)
        *yaw_l = (float)(atan2(*aim_y_l, *aim_x_l));
}

float aim_x_l = 0, aim_y_l = 0, aim_z_l = 0; // aim point 落点，传回上位机用于可视化
float pitch_l = 0; //输出控制量 pitch_l绝对角度 弧度
float yaw_l = 0;   //输出控制量 yaw_l绝对角度 弧度

// 从坐标轴正向看向原点，逆时针方向为正

trajectory_target_s *rv2_trajectory_init_L()
{
    //定义参数
    st_l.k = 0.092;
    st_l.bullet_type =  BULLET_17;
    st_l.current_v = 21;//18
    st_l.current_pitch = 0;
    st_l.current_yaw = 0;
    st_l.xw = 0.0;
    st_l.yw = 0;
    st_l.zw = 0;

    st_l.vxw = 0;
    st_l.vyw = 0;
    st_l.vzw = 0;
    st_l.v_yaw = 0;
    st_l.tar_yaw = 0;
    st_l.r1 = 0;
    st_l.r2 = 0;
    st_l.dz = 0;

    //以下设置参数
    st_l.bias_time = 10;
    st_l.s_bias = 0.07;  //0.2
    st_l.z_bias = 0.06;   //0.19
    st_l.armor_id = ARMOR_INFANTRY3;
    st_l.armor_num = ARMOR_NUM_NORMAL;

    // printf("main pitch_l:%f° yaw_l:%f° ", pitch_l * 180 / PI, yaw_l * 180 / PI);
    // printf("\npitch_l:%frad yaw_l:%frad aim_x_l:%f aim_y_l:%f aim_z_l:%f", pitch_l, yaw_l, aim_x_l, aim_y_l, aim_z_l);

    return &trajectory_target_l;
}

void rv2_trajectory_passin_L(rv2_recv_protocol_s *param1,float *param2)
{
    st_l.xw = param1->x;
    st_l.yw = param1->y;
    st_l.zw = param1->z;
    st_l.tar_yaw = param1->yaw;
    st_l.vxw = param1->vx;
    st_l.vyw = param1->vy;
    st_l.vzw = param1->vz;
    st_l.v_yaw = param1->v_yaw;
    st_l.r1 = param1->r1;
    st_l.r2 = param1->r2;
    st_l.dz = param1->dz;

    st_l.armor_id=param1->id;
    st_l.armor_num=param1->armors_num;

    st_l.current_pitch=param2[1];
    st_l.current_yaw = param2[0];


}

void rv2_trajectory_calculate_L()
{

    //预测
    autoSolveTrajectory_L(&pitch_l, &yaw_l, &aim_x_l, &aim_y_l, &aim_z_l);

    //返回值
    trajectory_target_l.pitch=pitch_l;
    trajectory_target_l.yaw=yaw_l;
    trajectory_target_l.aim_x=aim_x_l;
    trajectory_target_l.aim_y=aim_y_l;
    trajectory_target_l.aim_z=aim_z_l;
}


