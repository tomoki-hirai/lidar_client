/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <vector>
#include <algorithm>

#include <math.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

struct BuffInt {
	int i;
	int j;
};

struct scanDot {
            _u8   quality;
            float angle;
            float dist;
};

struct scanDot_Simple {
	int   cnt;
	float angle;
	float dist;
	float distX;  //mm単位の直交座標
	float distY;  //mm単位の直交座標
	int X;
	int Y;
};

struct scanDot_Complication :public scanDot_Simple {
	float previousangle;
	float previousdist;
	int previousX;
	int previousY;
	int uniqueid;
	int flag;
	int milliseconds;
	time_t now;
};

bool operator<(const scanDot& left, const scanDot& right)
{
	return left.angle < right.angle;
}

bool operator>(const scanDot& left, const scanDot& right)
{
	return left.angle > right.angle;
}
bool operator<(const scanDot_Complication& left, const scanDot_Complication& right)
{
	return left.cnt < right.cnt;
}
bool operator>(const scanDot_Complication& left, const scanDot_Complication& right)
{
	return left.cnt > right.cnt;
}


// ファイル読み込みに変更（position.txtに記述）
int _position = 1111;  
//ここの数字変える 0:104 中央 1:多目的 2:和室 3:配席付近 4:入口近く 5:プリンター手前 6:シスコの向かい 7:集中手前 99:111
// KC111の人流　:1111~1114
// KC111の在離席2000~2001
// KC111の在離席3001~3006

// 3x3lab:
int _UniqueId = _position * 100000;

enum position_one {
    position_one_X = -6000,
    position_one_Y = 2200,
};
enum position_two {
    position_two_X = -5800,
    position_two_Y = -2200,
};
enum position_three {
    position_three_X = 3900,
    position_three_Y = 3800,
};
enum position_four {
    position_four_X = 2400,
    position_four_Y = 1700,
};
enum position_five {
    position_five_X = -2400,
    position_five_Y = -2000,
};
enum position_six {
    position_six_X = 2100,
    position_six_Y = 2000,
};
enum position_seven {
    position_seven_X = 2500,
    position_seven_Y = -2100,
};

enum position_kc111one {
    position_kc111one_X = 3000,
    position_kc111one_Y = 1800,
};
enum position_kc111two {
    position_kc111two_X = -3000,
    position_kc111two_Y = -2400,
};
enum position_kc111three {
    position_kc111three_X = 1800,
    position_kc111three_Y = -200,
};
enum position_kc111four {
    position_kc111four_X = 2700,
    position_kc111four_Y = -1800,
};
enum position_kc111five {
    position_kc111five_X = -2400,
    position_kc111five_Y = 1200,
};
enum position_kc111six {
    position_kc111six_X = -1200,
    position_kc111six_Y = -600,
};

// 3x3lab用
enum position_3x3lab_1 {
    position_3x3lab_1_X = 0,
    position_3x3lab_1_Y = 0,
};
enum position_3x3lab_2 {
    position_3x3lab_2_X = 0,
    position_3x3lab_2_Y = 0,
};
enum position_3x3lab_3 {
    position_3x3lab_3_X = 0,
    position_3x3lab_3_Y = 0,
};
enum position_3x3lab_4 {
    position_3x3lab_4_X = 0,
    position_3x3lab_4_Y = 0,
};
enum position_3x3lab_5 {
    position_3x3lab_5_X = 0,
    position_3x3lab_5_Y = 0,
};
enum position_3x3lab_6 {
    position_3x3lab_6_X = 0,
    position_3x3lab_6_Y = 0,
};


class MoveArea
{
public:
	MoveArea();
	virtual ~MoveArea();
	bool checkMoveArea(float distX, float distY, int area);
};

MoveArea::MoveArea()
{
}
MoveArea::~MoveArea()
{
}
bool MoveArea::checkMoveArea(float distX, float distY, int area) {
	//条件の中ならTure,
	bool re_bool = false;

    // kc111のグリッドの大きさ
    int grid_x = 3680;
    int grid_y = 3040;
    //左側の余白
    int left_margin = 280;
    //左下の窪地
    //int left_holl = 220;

	if (area == 0) {
		if (-6300 < distX && distX < 8900) {  //kc104の部屋の中
			if (-4400 < distY && distY < 4000) {
				//個別の静止物
				if (4100 < distX && distX < 4500 && 0 < distY && distY < 1100);  //木(多目的)
				else if (-850 < distX && distX < -400 && -3200 < distY && distY < -2900);  //木(集中)
				else if (-2400 < distX && distX < 3600 && -3350 < distY && distY < -3200);  //ガラス(集中)
				else if (-2400 < distX && distX < -2200 && -5000 < distY && distY < -3350);  //ガラス(集中)
				else if (3400 < distX && distX < 3500 && -4500 < distY && distY < -3500);  //ガラス(集中)
				else if (8000 < distX  &&  3000< distY );  //多目的スペース
				else if (distX < -4400 && distY < 2850);  //ロッカー
				else    re_bool = true; //kc104の部屋の中かつ個別の静止物の中でない
			}
		}
		return re_bool;
	}else if (area == 1) {
		if(-9000 < distX && distX < 3000) {  //kc104の部屋の中
			if (-6000 < distY && distY < 2150) {
				//個別の静止物
				if (1990 < distX && 0 < distY);  //パーティション
				else if (-1700 < distX && distX < 1200 && -1800 < distY && distY < -1450);  //ホワイトボード
				else if (-3000 < distX && distX < -1600 && 1900 < distY && distY < 2200);  //柱
				else if (-2000 < distX && distX < -1500 && -1500 < distY && distY < -1000);  //木
				else    re_bool = true; //kc104の部屋の中かつ個別の静止物の中でない
			}
		}
		return re_bool;
	}else if (area == 2) {
		if (-9000 < distX && distX < 3200) {  //kc104の部屋の中
			if ( -2500 < distY && distY < 5900) {
				//個別の静止物
				if (-1800 < distX && distX < 1300 && 2900 < distY && distY < 3200 );  //ホワイトボード
				else if (-2400 < distX && distX < 200 && -2500 < distY && distY < -2300);  //デイスプレイ
				else if (-1650 < distX && distX < -1500 && 1030 < distY && distY < 1180);  //柱
				else if (-2450 < distX && distX < -2350 && -1050 < distY && distY < -900);  //集中
				else    re_bool = true; //kc104の部屋の中かつ個別の静止物の中でない
			}
		}
		return re_bool;
	}else if (area == 3) {
		if (-4300 < distX && distX < 9000) {  //kc104の部屋の中
			if (-9000 < distY && distY < 150) {
				//個別の静止物
				if (-4300 < distX && distX < -600 &&  distY < -1100);  //壁
				else if ( distX < -600 && distY < -1200);  //壁
				else    re_bool = true; //kc104の部屋の中かつ個別の静止物の中でない
			}
		}
		return re_bool;
	}else if (area == 99) {  //kc101 テスト用
		if (-15000 < distX && distX < 0) {  
			if (-500 < distY && distY < 800) {
				re_bool = true; //kc101の部屋の中かつ個別の静止物の中でない
			}
		}
		return re_bool;
	}
    else if (area == 1111) {  //kc111
		// if (-3900 + position_kc111one_X < distX && distX < 3500 + position_kc111one_X ) {
		// 	if (-3000  - position_kc111one_Y < distY && distY < 2700  - position_kc111one_Y ) {
		// 		re_bool = true; //kc111の部屋の中かつ個別の静止物の中でない
		// 	}
		// }
        float x_min = -grid_x - left_margin + 100 + position_kc111one_X;
        float x_max = grid_x - 200 + position_kc111one_X;

        float y_min = -grid_y - position_kc111one_Y + 100;
        float y_max = grid_y - position_kc111one_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
                printf("X:%f < %f\n",x_min,x_max);
                printf("Y:%f < %f\n",y_min,y_max);
                printf("check:%f,%f\n",distX,distY);
            }
        }

        // if (-grid_x - left_margin + 100 + position_kc111one_X < distX && distX < grid_x - 200 + position_kc111one_X ) {
		// 	if (-grid_y - position_kc111one_Y + 200 < distY && distY < grid_y - position_kc111one_Y - 800) {
		// 		re_bool = true; //kc111の部屋の中かつ個別の静止物の中でない
		// 	}
		// }
        // int temp_num = grid_y - position_kc111one_Y - 800;
        // printf("%d\n",temp_num);



		// if (2750 + position_kc111one_X  < distX && distX < 3750  + position_kc111one_X  && 2350  - position_kc111one_Y < distY && distY < 3220 - position_kc111one_Y ) re_bool = false;  //PC
        // if (-3900 + position_kc111one_X < distX && distX < -3000 + position_kc111one_X && 1050 - position_kc111one_Y < distY && distY < 2700 - position_kc111one_Y) re_bool = false;  //椅子

        //if (grid_x - 1000 + position_kc111one_X < distX && distX < grid_x + position_kc111one_X && grid_y - 1200 + position_kc111one_Y < distY && distY < grid_y + position_kc111one_Y) re_bool = false;  //PC
        //if (-grid_x - 1000 + position_kc111one_X < distX && distX < -grid_x + 500 + position_kc111one_X && grid_y - 1800 + position_kc111one_Y < distY && distY < grid_y + position_kc111one_Y) re_bool = false;  //椅子
        
        //棚
        // if (-grid_x + 300  + position_kc111one_X < distX && distX < -grid_x + 1100 + position_kc111one_X){
        //     if(grid_y - 500 - position_kc111one_Y < distY && distY < grid_y - position_kc111one_Y){
        //         re_bool = false;
        //     }
        // }
		return re_bool;
	}
    else if (area == 1112) {
        float x_min = -grid_x - left_margin + 100 + position_kc111two_X;
        float x_max = grid_x - 200 + position_kc111two_X;

        float y_min = -grid_y - position_kc111two_Y + 100;
        float y_max = grid_y - position_kc111two_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
                // printf("X:%f < %f\n",x_min,x_max);
                // printf("Y:%f < %f\n",y_min,y_max);
                // printf("check:%f,%f\n",distX,distY);
            }
        }
          //kc111
		// if (-3900 + position_kc111two_X < distX && distX < 3500 + position_kc111two_X ) {
		// 	if (-3000  - position_kc111two_Y < distY && distY < 2700  - position_kc111two_Y ) {
		// 		re_bool = true; //kc111の部屋の中かつ個別の静止物の中でない
		// 	}
		// }

    //    if (-grid_x - left_margin + 100 + position_kc111two_X < distX && distX < grid_x - 100 + position_kc111two_X ) {
	// 		if (-grid_y - position_kc111two_Y < distY && distY < grid_y - position_kc111two_Y ) {
	// 			re_bool = true; //kc111の部屋の中かつ個別の静止物の中でない
	// 		}
	// 	}

		// if (2750 + position_kc111two_X  < distX && distX < 3750  + position_kc111two_X  && 2350  - position_kc111two_Y < distY && distY < 3220 - position_kc111two_Y ) re_bool = false;  //PC
        // if (-3900 + position_kc111two_X < distX && distX < -3000 + position_kc111two_X && 1050 - position_kc111two_Y < distY && distY < 2700 - position_kc111two_Y) re_bool = false;  //椅子
		
        // if (grid_x - 1000 + position_kc111two_X < distX && distX < grid_x + position_kc111two_X && grid_y - 1200 + position_kc111two_Y < distY && distY < grid_y + position_kc111two_Y) re_bool = false;  //PC
        // if (-grid_x - 1000 + position_kc111two_X < distX && distX < -grid_x + 500 + position_kc111two_X && grid_y - 1800 + position_kc111two_Y < distY && distY < grid_y + position_kc111two_Y) re_bool = false;  //椅子
        // if (-grid_x + 100 + position_kc111two_X < distX && distX < -grid_x + 900 + position_kc111two_X && grid_y - 500 + position_kc111two_Y < distY && distY < grid_y + position_kc111two_Y) re_bool = false;  //棚

		return re_bool;
	}
    else if (area == 2000) {
        float x_min = -grid_x - left_margin + 100 + position_kc111three_X;
        float x_max = grid_x - 200 + position_kc111three_X;

        float y_min = -grid_y - position_kc111three_Y + 100;
        float y_max = grid_y - position_kc111three_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
                // printf("X:%f < %f\n",x_min,x_max);
                // printf("Y:%f < %f\n",y_min,y_max);
                // printf("check:%f,%f\n",distX,distY);
            }
        }
        
          //kc111 在離席
    // if (-grid_x - left_margin - position_kc111three_X < distX && distX < grid_x + position_kc111three_X) {
    //     if (-grid_y - position_kc111three_Y < distY && distY < grid_y + position_kc111three_Y) {
    //     re_bool = true; //kc111の部屋の中かつ個別の静止物の中でない
    //     }
    // }

    // 椅子
    if(x_min < distX && distX < x_min + 700){
            if(y_max - 2000 < distY && distY < y_max){
                re_bool = false;
            }
        }

    //if (grid_x - 1000 + position_kc111three_X < distX && distX < grid_x + position_kc111three_X && grid_y - 1200 + position_kc111three_Y < distY && distY < grid_y + position_kc111three_Y) re_bool = false;  //PC
    //if (-grid_x - 1000 + position_kc111three_X < distX && distX < -grid_x + 500 + position_kc111three_X && grid_y - 1800 + position_kc111three_Y < distY && distY < grid_y + position_kc111three_Y) re_bool = false;  //椅子
    //if (-grid_x + 100 + position_kc111three_X < distX && distX < -grid_x + 900 + position_kc111three_X && grid_y - 400 + position_kc111three_Y < distY && distY < grid_y + position_kc111three_Y) re_bool = false;  //棚

    return re_bool;
    }
    else if (area == 1113) {
        float x_min = -grid_x - left_margin + 100 + position_kc111four_X;
        float x_max = grid_x - 200 + position_kc111four_X;

        float y_min = -grid_y - position_kc111four_Y + 100;
        float y_max = grid_y - position_kc111four_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
            }
        }
        

    // 椅子
    if(x_min < distX && distX < x_min + 700){
            if(y_max - 2000 < distY && distY < y_max){
                re_bool = false;
            }
        }
    return re_bool;
    }
    else if (area == 1114) {
        float x_min = -grid_x - left_margin + 100 + position_kc111five_X;
        float x_max = grid_x - 200 + position_kc111five_X;

        float y_min = -grid_y - position_kc111five_Y + 100;
        float y_max = grid_y - position_kc111five_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
            }
        }
    // 椅子
    if(x_min < distX && distX < x_min + 700){
            if(y_max - 2000 < distY && distY < y_max){
                re_bool = false;
            }
        }
    return re_bool;
    }
    else if (area == 2001) {
        float x_min = -grid_x - left_margin + 100 + position_kc111six_X;
        float x_max = grid_x - 200 + position_kc111six_X;

        float y_min = -grid_y - position_kc111six_Y + 100;
        float y_max = grid_y - position_kc111six_Y - 800;

        if(x_min < distX && distX < x_max){
            if(y_min < distY && distY < y_max){
                re_bool = true;
            }
        }
    // 椅子
    if(x_min < distX && distX < x_min + 700){
            if(y_max - 2000 < distY && distY < y_max){
                re_bool = false;
            }
        }
    return re_bool;
    }
	else {
		return re_bool;
	}
    
}

const float PI   = (float)3.14159265;
const int SCAN_DATA_SIZE = 1; 
const int OBJ_CENTER_SIZE = 1;
const int OBJ_SEMI_SIZE = 1;
const int OBJ_MOVE_SIZE = 40;
const int OBJ_MOVE_FEW = 10;
const int OBJ_MOVE_MANY = 100;
const int MINIMUM_ELE_NUM = 2;
const int DEF_DISTANCE = 300;

std::vector<scanDot> _scan_data[SCAN_DATA_SIZE];
std::vector<scanDot_Simple> _obj_center[OBJ_CENTER_SIZE];
std::vector<scanDot_Complication> _obj_move[OBJ_MOVE_SIZE];
std::vector<scanDot_Complication> _obj_move_log;
std::vector<scanDot_Complication> _obj_semi_move[OBJ_SEMI_SIZE];

MoveArea m_movearea;



void OnObjSum(scanDot_Simple *objsum, float dist, float angle, bool IfTrueppElone) {
	//中心計算用のsum
	if (IfTrueppElone){
		objsum->dist += dist;
		objsum->angle += angle;
		objsum->cnt += 1;
	}
	else {
		objsum->dist = dist;
		objsum->angle = angle;
		objsum->cnt = 1;
	}
}

void OnObjCenterPush(scanDot_Simple *objsum) {
	//距離と角度の合計もらって中心計算，_obj_centerにpushする
	scanDot_Simple tmp_scan;
	tmp_scan.dist = objsum->dist / float(objsum->cnt);
	tmp_scan.angle = objsum->angle / float(objsum->cnt);
	tmp_scan.cnt = 0;
	_obj_center[0].push_back(tmp_scan);
	objsum->cnt = 0;  //OnObjCenterPushのflagになる
}

void OnCopy(std::vector<scanDot_Complication> *obj, int i, int j) {
    //物体中心のデータを移動物体にコピー，iは物体中心，jは移動物体
	obj[0][j].previousangle = obj[0][j].angle;
	obj[0][j].previousdist = obj[0][j].dist;
	obj[0][j].previousX = obj[0][j].X;
	obj[0][j].previousY = obj[0][j].Y;
	obj[0][j].angle = _obj_center[0][i].angle;
	obj[0][j].dist = _obj_center[0][i].dist;
	obj[0][j].X = _obj_center[0][i].X;
	obj[0][j].Y = _obj_center[0][i].Y;
}

void OnObjCntUpdata(std::vector<scanDot_Complication> *obj, int j, bool boo) { 
    //trueが+に変更，falseが-に変更
	if (obj[0][j].cnt >= 0) {
		if (boo == true) obj[0][j].cnt++;
		else obj[0][j].cnt = -1;
	}
	else {
		if (boo == true) obj[0][j].cnt = 1;
		else obj[0][j].cnt--;
	}
}

void OnMoveComplicationBase(scanDot_Complication *tmp_scan, int i){
	tmp_scan->previousangle = _obj_center[0][i].angle;
	tmp_scan->previousdist = _obj_center[0][i].dist;
	tmp_scan->previousX = _obj_center[0][i].X;
	tmp_scan->previousY = _obj_center[0][i].Y;
	tmp_scan->angle = _obj_center[0][i].angle;
	tmp_scan->dist = _obj_center[0][i].dist;
	tmp_scan->X = _obj_center[0][i].X;
	tmp_scan->Y = _obj_center[0][i].Y;
	tmp_scan->cnt = 0;
	tmp_scan->uniqueid = 1000;
	tmp_scan->flag = 0;   
	// tmp_scan->now = std::time(0); 
	// SYSTEMTIME st;
	// GetLocalTime(&st);
	// tmp_scan->milliseconds = st.wMilliseconds;
}

inline void OnMoveComplicationMake(int i, std::vector<scanDot_Complication> *obj) {
	scanDot_Complication tmp_scan;
	OnMoveComplicationBase(&tmp_scan, i);
	obj[0].push_back(tmp_scan);
}

void OnPositionGloble(int positionX, int positionY, std::vector<scanDot_Complication> &_obj) {
	float rad = (float)(_obj.back().angle * PI / 180.0);
	float distX = sin(rad)*(_obj.back().dist) - positionX;
	float distY = cos(rad)*(_obj.back().dist) + positionY;
	float previousrad = (float)(_obj.back().previousangle * PI / 180.0);
	float previousdistX = sin(previousrad)*(_obj.back().previousdist) - positionX;
	float previousdistY = cos(previousrad)*(_obj.back().previousdist) + positionY;
	if (distX > 0) _obj.back().angle = (PI / 2.0 - atan(distY / distX)) * 180 / PI;
	else           _obj.back().angle = (PI + PI / 2.0 - atan(distY / distX)) * 180 / PI;
	_obj.back().dist = sqrtf(std::pow(distX, 2) + std::pow(distY, 2));
	if(previousdistX > 0) _obj.back().previousangle = (PI / 2.0 - atan(previousdistY / previousdistX)) * 180 / PI;
	else                  _obj.back().previousangle = (PI + PI / 2.0 - atan(previousdistY / previousdistX)) * 180 / PI;
	_obj.back().previousdist = sqrtf(std::pow(previousdistX, 2) + std::pow(previousdistY, 2));
}


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int readPosition(){
    std::ifstream ifs("./position.txt");
    std::string str;
    if (ifs.fail())
    {
        std::cerr << "position.txtが見つかりません" << std::endl;
        return 1;
    }
    getline(ifs, str);
    std::cout << "[" << str << "]" << std::endl;
    int num = atoi(str.c_str());
    return num;
}

int main(int argc, const char * argv[]) {
    printf("%d",_position);
    _position = readPosition();
    _UniqueId = _position * 100000;
    printf("%d",_position);
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    int sockfd;
    struct sockaddr_in addr;
    
    char buf[100];
    char bufall[2000];
    
    int bufuniqueid;
    double bufdist;
    double bufangle;

    if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) < 0){
        perror("socket");
    }
    
    addr.sin_family = AF_INET;       
    addr.sin_port = htons(34567);
    addr.sin_addr.s_addr = inet_addr("172.20.11.238");
    printf("before co\n");
    connect(sockfd,(struct sockaddr*)&addr,sizeof(struct sockaddr_in));
    printf("after co\n");

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
    "Version: " RPLIDAR_SDK_VERSION "\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    rplidar_response_device_info_t devinfo;

	// retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

	signal(SIGINT, ctrlc);
    
	drv->startMotor();
    // start scan...
    drv->startScan();

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                scanDot dot;
                if (!nodes[pos].distance_q2) continue;

                dot.quality = (nodes[pos].sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                dot.angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                dot.dist = nodes[pos].distance_q2/4.0f;
                _scan_data[0].push_back(dot);
            }
            std::sort(_scan_data[0].begin(), _scan_data[0].end());
        }

        scanDot_Simple objsum;
	    objsum.angle = 0;
        objsum.dist = 0;
        objsum.cnt = 0;
 
        float distance;

        //_scan_dataを_obj_centerに変換開始
        for (int pos = 0; pos < (int)_scan_data[0].size(); ++pos) {
            
            //LiDARが取得した極座標
            float dist = _scan_data[0][pos].dist;
            float angle = _scan_data[0][pos].angle;
            
            //極座標を直交座標に変換
            float rad = (float)(angle*PI / 180.0);
            float distX = sin(rad)*(dist);
            float distY = cos(rad)*(dist);

            //更新する直交座標を保存する
            float distXaf;
            float distYaf;

            // xyはscan_dataから算出
            if (pos != (int)_scan_data[0].size() - 1) {
                //一個先との差
                distXaf = sin((float)(_scan_data[0][pos + 1].angle*PI / 180.0))*_scan_data[0][pos + 1].dist;
                distYaf = cos((float)(_scan_data[0][pos + 1].angle*PI / 180.0))*_scan_data[0][pos + 1].dist;

                distance = sqrtf(std::pow(distX - distXaf, 2) + std::pow(distY - distYaf, 2));

                //設定したエリア内のうち静止物体でなければ
                // if (distance < DEF_DISTANCE  &&  m_movearea.checkMoveArea(distXaf, distYaf, _position) == true) {

                // 固定物削除は行わない
                if (distance < DEF_DISTANCE) {
                    if (objsum.cnt == 0) OnObjSum(&objsum, dist, angle, false);
                    else                 OnObjSum(&objsum, dist, angle, true);
                }
                else {
                    // if (objsum.cnt < MINIMUM_ELE_NUM && pos >= MINIMUM_ELE_NUM) {		//点の数が一定以下のとき，カウントしない
                    if (objsum.cnt < MINIMUM_ELE_NUM) {		//点の数が一定以下のとき，カウントしない
                        objsum.cnt = 0; //flag
                    }
                    else {
                        //OnObjSum(&objsum, dist, angle, true);
                        OnObjCenterPush(&objsum);
                    }
                }
            }
            //取得した極座標のうち最後を最初の極座標と合わせるように計算処理
            else if (pos == (int)_scan_data[0].size() - 1 && _obj_center[0].size() > 0) {
               // if (objsum.cnt == 0) OnObjSum(&objsum, dist, angle, false);
               // else                 OnObjSum(&objsum, dist, angle, true);

                OnObjCenterPush(&objsum);
                
                //最後に取得した極座標のデータ
                float radend = (float)(_obj_center[0].back().angle*PI / 180.0);	
                float distXend = sin(radend)*(_obj_center[0].back().dist);
                float distYend = cos(radend)*(_obj_center[0].back().dist);
                
                //最初に取得した極座標のデータ
                float radzero = (float)(_obj_center[0][0].angle*PI / 180.0);
                float distXzero = sin(radzero)*(_obj_center[0][0].dist);
                float distYzero = cos(radzero)*(_obj_center[0][0].dist);
                
                float distance_center = sqrtf(std::pow(distXend - distXzero, 2) + std::pow(distYend - distYzero, 2));
                
                //最初と最後のつながり
                if (distance_center < 900) { //5000*sin(10*PI()/180)
                    scanDot_Simple tmpscan;
                    tmpscan.dist = (_obj_center[0][0].dist + _obj_center[0].back().dist) / 2;
                    tmpscan.angle = (_obj_center[0][0].angle + (_obj_center[0].back().angle + 360)) / 2;
                    if (tmpscan.angle > 360) {
                        tmpscan.angle -= 360;
                    }

                    tmpscan.cnt = 0;
            
                    try {
                        _obj_center[0].pop_back();  //2つをつなげて新しいの作る，古いの消す
                        _obj_center[0].push_back(tmpscan);
                    }
                    catch (std::exception &e) {
                        std::cout<<"*********1*********";
                        std::cout << e.what() << std::endl;
                    }
                    std::swap(_obj_center[0][0], _obj_center[0].back());
                    try {
                        _obj_center[0].pop_back();
                    }
                    catch (std::exception &e) {
                        std::cout << "*********2*********";
                        std::cout << e.what() << std::endl;
                    }
                }
                else {
                }
            }
        }
        //_scan_dataを_obj_centerに変換終了




            //追跡
            std::vector<int> buffmove;
            std::vector<int> buffmovekill;
            std::vector<BuffInt> buffsemi;
            int pop_back_cnt = 0;

            //_obj_centerを_obj_moveに変換開始
            //整形したLiDARからのデータのうち移動物体とみなした座標を抽出

            //条件に合った_obj_moveをkillする
            for (int j = (int)_obj_move[0].size()-1; j >= 0; --j) {
                try {
                    //静止物体キル
                    if (_obj_move[0].at(j).cnt < -50) {
                        std::swap(_obj_move[0][j], _obj_move[0].back());
                        _obj_move[0].pop_back();
                    }
                }
                catch (std::exception &e) {
                    std::cout << "*********_move_kill*********";
                    std::cout << e.what() << std::endl;
                }
            }
            
            //_obj_centerと_obj_moveを対応づけ
            for (int j = 0; j < (int)_obj_move[0].size() ; ++j) {
                float distance;
                float best_distance = 300;  //テキトー
                int best_center_ele = -1;

                float radmove = (float)(_obj_move[0][j].angle*PI / 180.0);
                float distXmove = sin(radmove)*(_obj_move[0][j].dist);
                float distYmove = cos(radmove)*(_obj_move[0][j].dist);

                //nowの更新
                // _obj_move[0][j].now = std::time(0); 
                // SYSTEMTIME st;
                // GetLocalTime(&st);
                // _obj_move[0][j].milliseconds = st.wMilliseconds;

                for (int i = 0; i < (int)_obj_center[0].size(); i++) {
                    //cntは使ったかどうかのflag
                    if (_obj_center[0][i].cnt != 1) { 
                        //簡易追跡
                        float radcenter = (float)(_obj_center[0][i].angle*PI / 180.0);
                        float distXcenter = sin(radcenter)*(_obj_center[0][i].dist);
                        float distYcenter = cos(radcenter)*(_obj_center[0][i].dist);

                        distance = sqrtf(std::pow(distXmove - distXcenter, 2)+ std::pow(distYmove - distYcenter, 2));
                        
                        //moveから一番近いcenter
                        if (distance < best_distance) {  
                            best_distance = distance;
                            best_center_ele = i;
                        }
                    }
                }
                
                //一番近いのに_obj_moveを更新
                if (best_center_ele > -1) {  
                    OnCopy(_obj_move, best_center_ele, j);
                    //cntは使ったかどうかのflag
                    _obj_center[0][best_center_ele].cnt = 1;
                    //_obj_move[0][j].cnt++;  と同じcntは連続データ更新の回数  
                    OnObjCntUpdata(_obj_move, j, true);  
                }
                else {
                    OnObjCntUpdata(_obj_move, j, false);
                    //_obj_move使ってないリスト
                    buffmove.push_back(j);  
                }
            }

            //_obj_semi_moveと_obj_centerを対応づけ
            for (int j = 0; j < (int)_obj_semi_move[0].size(); j++) {  
                float distance;
                float best_distance = 300;  //テキトー
                int best_center_ele = -1;

                float radsemi = (float)(_obj_semi_move[0][j].angle*PI / 180.0);
                float distXsemi = sin(radsemi)*(_obj_semi_move[0][j].dist);
                float distYsemi = cos(radsemi)*(_obj_semi_move[0][j].dist);
                
                for (int i = 0; i < (int)_obj_center[0].size(); i++) {
                    //_obj_centerの使ってないやつ　0:未使用　1:使用済み
                    if (_obj_center[0][i].cnt != 1) {  
                        float radcenter = (float)(_obj_center[0][i].angle*PI / 180.0);
                        float distXcenter = sin(radcenter)*(_obj_center[0][i].dist);
                        float distYcenter = cos(radcenter)*(_obj_center[0][i].dist);
                        distance = sqrtf(std::pow(distXsemi-distXcenter, 2)+ std::pow(distYsemi - distYcenter, 2));
                        
                        //_obj_centerと_obj_semi_moveの一番近いの
                        if (distance < best_distance) {  
                            best_distance = distance;
                            best_center_ele = i;
                        }
                    }
                }

                if (best_center_ele > -1) {
                    OnCopy(_obj_semi_move, best_center_ele, j);  //一番近いのに_obj_semi_moveを更新
                    _obj_center[0][best_center_ele].cnt = 1;  //cntは使ったかどうかのflag
                    _obj_semi_move[0][j].cnt++;    //cntは連続データ更新の回数
                
                    //一定回連続で更新されたとき
                    if (_obj_semi_move[0][j].cnt > 1) {  
                        BuffInt tmp;
                        tmp.i = best_center_ele;
                        tmp.j = j;
                        //_obj_move生成用に_obj_semi_moveのデータを保持
                        buffsemi.push_back(tmp);  
                    }
                }
                else {
                    _obj_semi_move[0][j].cnt = 1000;  //削除処理用
                    pop_back_cnt++;  //削除する個数
                }
            }
            //未使用の_obj_moveと条件を満たした_obj_semi_moveを対応づけ，出来なければ_obj_moveを新規生成
            for (int j = 0; j < (int)buffsemi.size(); j++) {  
                float distance;
                float best_distance = 2000;  //テキトー
                int best_ele = -1;
            
                float radbuffsemi = (float)(_obj_semi_move[0][buffsemi[j].j].angle*PI / 180.0);
                float distXbuffsemi = sin(radbuffsemi)*(_obj_semi_move[0][buffsemi[j].j].dist);
                float distYbuffsemi = cos(radbuffsemi)*(_obj_semi_move[0][buffsemi[j].j].dist);
            
                //未使用の_obj_moveと条件を満たした_obj_semi_moveを対応づけ
                for (int i = 0; i < (int)buffmove.size(); i++) {  
                    if (_obj_move[0][buffmove[i]].cnt > -30) {
                        float radmove = (float)(_obj_move[0][buffmove[i]].angle*PI / 180.0);
                        float distXmove = sin(radmove)*(_obj_move[0][buffmove[i]].dist);
                        float distYmove = cos(radmove)*(_obj_move[0][buffmove[i]].dist);
                        distance = sqrtf(std::pow(distXbuffsemi - distXmove, 2)+ std::pow(distYbuffsemi - distYmove, 2));
            
                        if (distance < best_distance) {
                            best_distance = distance;
                            best_ele = i;
                        }
                    }
                }
            
                //_obj_moveを_obj_semi_moveを対応づけ
                if (best_ele > -1) {  
                    OnCopy(_obj_move, buffsemi[j].i, buffmove[best_ele]);
                }
                //_obj_semi_moveの位置に_obj_moveを生成
                else {  
                    OnMoveComplicationMake(buffsemi[j].i, _obj_move);
                    _obj_move[0].back().uniqueid = _UniqueId;
                    _UniqueId++;  //uniqueIdを入れる
                }
                _obj_semi_move[0][buffsemi[j].j].cnt = 1000;  //削除処理用
                pop_back_cnt++;  //削除する個数
            }
            
            std::sort(_obj_semi_move[0].begin(), _obj_semi_move[0].end());
            for (int i = pop_back_cnt; i > 0; --i) {
                try {
                    if (_obj_semi_move[0].at(_obj_semi_move[0].size() - 1).cnt >= 1000) {
                        _obj_semi_move[0].at(_obj_semi_move[0].size() - 1);
                        _obj_semi_move[0].pop_back();
                    }
                }
                catch (std::exception &e) {
                    std::cout << "*********4*********";
                    std::cout << e.what() << std::endl;
                }
            }
            
            for (int i = 0; i < (int)_obj_center[0].size(); i++) {
                if (_obj_center[0][i].cnt != 1) {
                    OnMoveComplicationMake(i, _obj_semi_move);
                }
            }
            //_obj_centerを_obj_moveに変換終了
                        
            //_obj_moveを_obj_sendに変換開始            
            std::vector<scanDot_Complication> _obj_send;
            memset(bufall,0,sizeof(bufall));
            for (int i = 0; i < (int)_obj_move[0].size(); ++i) {
                _obj_send.push_back(_obj_move[0][i]);  //sendデータ作成
                
                //_positionに応じた処理
                // if      (_position == 1) OnPositionGloble(position_one_X, position_one_Y, _obj_send);
                // else if (_position == 2) OnPositionGloble(position_two_X, position_two_Y, _obj_send);
                // else if (_position == 3) OnPositionGloble(position_three_X, position_three_Y, _obj_send);
                // else if (_position == 4) OnPositionGloble(position_four_X, position_four_Y, _obj_send);
                // else if (_position == 5) OnPositionGloble(position_five_X, position_five_Y, _obj_send);
                // else if (_position == 6) OnPositionGloble(position_six_X, position_six_Y, _obj_send);
                // else if (_position == 7) OnPositionGloble(position_seven_X, position_seven_Y, _obj_send);
                // else if (_position == 1111) OnPositionGloble(position_kc111one_X, position_kc111one_Y, _obj_send);
                // else if (_position == 1112) OnPositionGloble(position_kc111two_X, position_kc111two_Y, _obj_send);
                // else if (_position == 2000) OnPositionGloble(position_kc111three_X, position_kc111three_Y, _obj_send);
                // else if (_position == 1113) OnPositionGloble(position_kc111four_X, position_kc111four_Y, _obj_send);
                // else if (_position == 1114) OnPositionGloble(position_kc111five_X, position_kc111five_Y, _obj_send);
                // else if (_position == 2001) OnPositionGloble(position_kc111six_X, position_kc111six_Y, _obj_send);
                // // 3x3lab
                // else if (_position == 3001) OnPositionGloble(position_3x3lab_1_X, position_3x3lab_1_Y, _obj_send);
                // else if (_position == 3002) OnPositionGloble(position_3x3lab_2_X, position_3x3lab_2_Y, _obj_send);
                // else if (_position == 3003) OnPositionGloble(position_3x3lab_3_X, position_3x3lab_3_Y, _obj_send);
                // else if (_position == 3004) OnPositionGloble(position_3x3lab_4_X, position_3x3lab_4_Y, _obj_send);
                // else if (_position == 3005) OnPositionGloble(position_3x3lab_5_X, position_3x3lab_5_Y, _obj_send);
                // else if (_position == 3006) OnPositionGloble(position_3x3lab_6_X, position_3x3lab_6_Y, _obj_send);
                
                //printf("番号：%d 角度：%03.2f 距離：%08.2f\n", _obj_send[i].uniqueid , _obj_send[i].angle , _obj_send[i].dist);

                bufuniqueid = _obj_send[i].uniqueid;
                bufdist = _obj_send[i].dist;
                bufangle = _obj_send[i].angle;
                
                memset(buf,0,sizeof(buf));
                sprintf(buf,"%d,%d,%d,",bufuniqueid,(int)bufdist,(int)(bufangle)*100);
                
                if(i == 0){
                    strcpy(bufall,buf);
                }else{
                    strcat(bufall,buf);
                }

            }
            printf("%s\n",bufall);
            send(sockfd,(const char *)(&bufall),sizeof(bufall),0);

            //_obj_moveを_obj_sendに変換終了
            
            //各データの初期化
            //_scan_dataの処理
            for (int i = SCAN_DATA_SIZE - 1; i > 0; --i) {  
                _scan_data[i].swap(_scan_data[i - 1]);
            }
            
            _scan_data[0].clear();
            _obj_center[0].clear();
            //_obj_moveの処理
            for (int i = OBJ_MOVE_SIZE - 1; i > 1; --i) { //1～9まで
                _obj_move[i].swap(_obj_move[i - 1]);
            }
            
            _obj_move[1].clear(); //あってる
            
            for (int i = 0; i < (int)_obj_move[0].size(); ++i) {
                _obj_move[1].push_back(_obj_move[0][i]);
            }
    

        if (ctrl_c_pressed){ 
            break;
        }

    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

