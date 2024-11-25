#pragma once
#ifndef GNG_H
#define GNG_H
/*
 *  gng.h
 *  Cluster
 *
 *  Created by Yuichiro Toda on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */

#include <random>
#include <thread>
#include <vector>
#include <limits>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define GNGN 600		//最大ノード数def5000
#define ND 300		    //データ数
#define DIM 6			//ベクトルの次元数
#define LDIM 4			//学習を行うベクトルの次元数
#define NOP 4			//構築するクラスタの種類の数
//#define MAXANGLE 20
#define THV 0.001*0.001	//def 20*20
#define DIS_THV 0.5

struct gng {

	double** node;		//ノード用配列
	int** edge;			//エッジ用配列（位置情報）
	int** cedge;		//エッジ用配列（色情報）
	int** nedge;		//エッジ用配列（法線情報）
	int** age;			//エッジの年齢用配列
	int* edge_ct;       //エッジの個数カウント用
	int node_n;			//現在のノード数
	/*int cluster[NOP][GNGN][GNGN];
	int cluster2[NOP][GNGN];
	int cluster_num[NOP][GNGN];*/
	//double cluster_cog[NOP][GNGN][DIM];
	int cluster_ct[NOP];
	int pre_cluster_ct[NOP];
	double* gng_err;	//積算誤差用配列
	double* gng_u;		//utility valiables

	//double disu[GNGN];
	double dise[GNGN];
	double max_dise;
	double disrate;

	double cthv;
	double nthv;

};

static std::shared_ptr<std::vector<float>> min_distances;
static std::shared_ptr<std::vector<int>> selected_indices;
static int selected_indices_del_num;
static double dis_rad_min = 1.0;
static double dis_rad_max = 2.0;
static double dis_rad = 1.0;

static bool is_first_selection = true; // 最初の選択かどうかを追跡
static int learn_t_index = 0;
static bool FPS_1th = true;
static int FPS_Max = GNGN;


//2次元配列の開放
void free2d_double(double** a);
//2次元配列の初期化
double** malloc2d_double(int x, int y);
//2次元配列の開放
void free2d_int(int** a);
//2次元配列の初期化
int** malloc2d_int(int x, int y);
struct gng* init_gng();
void free_gng(struct gng* net);

#endif // GNG_H