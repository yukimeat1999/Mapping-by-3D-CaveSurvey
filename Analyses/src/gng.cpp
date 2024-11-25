/*
 *  gng.c
 *  Claster
 *
 *  Created by Yuichiro Toda on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */


#include "gng.h"
#include "pca.h"
#include "rnd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>

 //2次元配列の開放
void free2d_double(double** a)
{
	free(a[0]);
	free(a);
}
//2次元配列の初期化
double** malloc2d_double(int x, int y)
{
	double** a;
	int i;
	a = (double**)malloc(sizeof(double*) * (x + 1));
	a[0] = (double*)malloc(sizeof(double) * (y + 1) * (x + 1));
	for (i = 1; i < (x + 1); i++) a[i] = a[0] + i * (y + 1);
	memset(a[0], 0, sizeof(*a[0]));
	return a;
}

//2次元配列の開放
void free2d_int(int** a)
{
	free(a[0]);
	free(a);
}
//2次元配列の初期化
int** malloc2d_int(int x, int y)
{
	int** a;
	int i;
	a = (int**)malloc(sizeof(int*) * (x + 1));
	a[0] = (int*)malloc(sizeof(int) * (y + 1) * (x + 1));
	for (i = 1; i < (x + 1); i++) a[i] = a[0] + i * (y + 1);
	memset(a[0], 0, sizeof(*a[0]));
	return a;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

struct gng* init_gng()
{
	int i, j;
	struct gng* net = NULL;
	if (net == NULL)
		net = (struct gng*)malloc(sizeof(struct gng));

	net->node = malloc2d_double(GNGN, DIM);
	net->age = malloc2d_int(GNGN, GNGN);
	net->edge = malloc2d_int(GNGN, GNGN);
	net->cedge = malloc2d_int(GNGN, GNGN);
	net->nedge = malloc2d_int(GNGN, GNGN);
	
	net->gng_err = (double*)malloc(sizeof(double) * GNGN);
	net->gng_u = (double*)malloc(sizeof(double) * GNGN);

	net->edge_ct = (int*)malloc(sizeof(int) * GNGN);


	for (i = 0; i < GNGN; i++) {
		net->edge_ct[i] = 0;
		for (j = 0; j < GNGN; j++) {
			net->edge[i][j] = 0;
			net->cedge[i][j] = 0;
			net->nedge[i][j] = 0;
			net->age[i][j] = 0;
		}
	}
	for (i = 0; i < 2; i++) {
		net->edge_ct[i] = 1;
		for (j = 0; j < 2; j++) {
			if (i != j) {
				net->edge[i][j] = 1;
				net->cedge[i][j] = 1;
				net->nedge[i][j] = 1;
				
			}
		}
	}
	for (i = 0; i < GNGN; i++) {
		for(int j=0;j<LDIM;j++){
			net->node[i][j] = rnd();
		}

		for(int j=LDIM;j<DIM;j++){
			net->node[i][j] = 0.0;
		}
		net->node[i][9] = -10.0;
		net->gng_err[i] = 0.0;
		net->gng_u[i] = 0.0;
		//net->disu[i] = 0.0005;
		net->dise[i] = 0.0005;
	}

	net->max_dise = 0.0005;
	net->disrate = 1.0;
	net->cthv = 0.05;
	net->nthv = 0.998;
	net->node_n = 2;

	return net;
}

struct gng* m_init_gng()
{
	int i, j;
	struct gng* net = NULL;
	if (net == NULL)
		net = (struct gng*)malloc(sizeof(struct gng));

	net->node = malloc2d_double(GNGN, DIM);
	net->age = malloc2d_int(GNGN, GNGN);
	net->edge = malloc2d_int(GNGN, GNGN);
	net->cedge = malloc2d_int(GNGN, GNGN);
	net->nedge = malloc2d_int(GNGN, GNGN);

	net->gng_err = (double*)malloc(sizeof(double) * GNGN);
	net->gng_u = (double*)malloc(sizeof(double) * GNGN);

	net->edge_ct = (int*)malloc(sizeof(int) * GNGN);


	for (i = 0; i < GNGN; i++) {
		net->edge_ct[i] = 0;
		for (j = 0; j < GNGN; j++) {
			net->edge[i][j] = 0;
			net->cedge[i][j] = 0;
			net->nedge[i][j] = 0;
			net->age[i][j] = 0;
		}
	}
	for (i = 0; i < GNGN; i++) {
		for (int j = 0; j < LDIM; j++) {
			net->node[i][j] = 0.0;
		}

		for (int j = LDIM; j < DIM; j++) {
			net->node[i][j] = 0.0;
		}
		net->node[i][9] = -10.0;
		net->gng_err[i] = 0.0;
		net->gng_u[i] = 0.0;
		net->dise[i] = 0.0005;
	}

	net->max_dise = 0.0005;
	net->disrate = 1.0;
	net->cthv = 0.05;
	net->nthv = 0.998;
	net->node_n = 0;

	return net;
}

void free_gng(struct gng* net)
{
	free2d_int(net->age);
	free2d_int(net->edge);
    free2d_int(net->cedge);
    free2d_int(net->nedge);
	free2d_double(net->node);
	free(net->gng_err);
	free(net->gng_u);
	free(net);
}

//void gng_clustering(struct gng* net, int property)
//{
//	int i, j, k, n;
//	int edge_info;
//	int sflag = 0;
//	int flag[GNGN];
//	int c = 0; //ラベルの番号
//	int c_n = 0; //ラベルに含まれるノードの数
//	int sum = 0;
//	int sum_nv = 0;
//	
//	for (i = 0; i < net->node_n; i++) {
//		flag[i] = 0;
//	}
//
//	net->cluster[property][c][c_n] = 0;
//	net->cluster2[property][0] = c;
//
//	for (i = 0; i < net->node_n; i++) {
//		if (flag[i] == 0) {
//			net->cluster[property][c][c_n] = i;
//			net->cluster2[property][i] = c;
//			flag[i] = 1;
//			for (k = 0; k < DIM; k++) {
//				net->cluster_cog[property][c][k] = 0;
//				if (k < LDIM || net->node[i][7] != -10) {
//					net->cluster_cog[property][c][k] += net->node[i][k];
//				}
//			}
//			if (net->node[i][7] != -10) sum_nv++;
//			c_n++;
//			break;
//		}
//	}
//
//	while (sum != net->node_n) {
//		for (i = 0; i < c_n; i++) {
//			n = net->cluster[property][c][i];
//
//			for (j = 0; j < net->node_n; j++) {
//				if (property == 0) edge_info = net->edge[n][j];
//				else if (property == 1) edge_info = net->cedge[n][j];
//				else if (property == 2) edge_info = net->nedge[n][j];
//				else if (property == 3) edge_info = net->cedge[n][j] * net->nedge[n][j];
//				
//				if (n != j && edge_info == 1) {
//					sflag = 0;
//					for (k = 0; k < c_n; k++)
//						if (j == net->cluster[property][c][k])
//							sflag = 1;
//					if (sflag == 0) {
//						net->cluster[property][c][c_n] = j;
//						net->cluster2[property][j] = c;
//						flag[j] = 1;
//						for (k = 0; k < DIM; k++) {
//							if (k < LDIM || net->node[j][7] != -10) {
//								net->cluster_cog[property][c][k] += net->node[j][k];
//							}
//						}
//						if (net->node[j][7] != -10)
//							sum_nv++;
//						c_n++;
//					}
//				}
//			}
//		}
//		sum += c_n;
//
//		net->cluster_num[property][c] = c_n;
//		for (k = 0; k < LDIM; k++) {
//			if (c_n != 0)
//				net->cluster_cog[property][c][k] /= (double)c_n;
//		}
//		for (k = LDIM; k < DIM; k++) {
//			if (sum_nv != 0)
//				net->cluster_cog[property][c][k] /= (double)sum_nv;
//		}
//
//		c_n = 0;
//		sum_nv = 0;
//		c++; //ラベルの番号を増やす
//		for (i = 0; i < net->node_n; i++) {
//			if (flag[i] == 0) {
//				net->cluster[property][c][c_n] = i;
//				net->cluster2[property][i] = c;
//				flag[i] = 1;
//				for (k = 0; k < DIM; k++) {
//					net->cluster_cog[property][c][k] = 0;
//					if (k < LDIM || net->node[i][7] != -10) {
//						net->cluster_cog[property][c][k] += net->node[i][k];
//					}
//				}
//				if (net->node[i][7] != -10) sum_nv++;
//				c_n++;
//				break;
//			}
//		}
//
//	}
//
//	net->cluster_ct[property] = c;
//
//}

void discount_err_gng(struct gng* net)
{
	int i;
	//全てのノードの積算誤差の減少
	for (i = 0; i < net->node_n; i++) {
		net->gng_err[i] -= net->dise[i] * net->gng_err[i];
		net->gng_u[i] -= net->dise[i] * net->gng_u[i];
		if (net->gng_err[i] < 0)
			net->gng_err[i] = 0.0;

		if (net->gng_u[i] < 0)
			net->gng_u[i] = 0.0;
	}
}

void node_delete(struct gng *net, int del_num)
{
    int l;
    int last_num = net->node_n-1;
	int del_list[100];
	int del_ct = 0;
    
    for(int i=0;i<DIM;i++){
        net->node[del_num][i] = net->node[last_num][i];
        net->node[last_num][i] = 0.0;
    }
    
    net->gng_err[del_num] = net->gng_err[last_num];
    net->gng_err[last_num] = 0.0;
    
    net->gng_u[del_num] = net->gng_u[last_num];
    net->gng_u[last_num] = 0.0;
    
    net->dise[del_num] = net->dise[last_num];
    
    net->edge_ct[del_num] = net->edge_ct[last_num];
    net->edge_ct[last_num] = 0;
    
   /* for(l=0;l<NOP;l++){
        net->cluster2[l][del_num] = net->cluster2[l][last_num];
        net->cluster2[l][last_num] = 0;
    }*/
    
    for(int i=0;i<net->node_n-1;i++){
        net->age[del_num][i] = net->age[last_num][i];
        net->age[i][del_num] = net->age[i][last_num];
        net->age[last_num][i] = 0;
        net->age[i][last_num] = 0;
        
        if(net->edge[del_num][i] == 1){
            net->edge[del_num][i] = 0;
            net->edge[i][del_num] = 0;
            net->edge_ct[i]--;
			if(net->edge_ct[i] == 0){
				del_list[del_ct++] = i;
			}
        }
        
        net->edge[del_num][i] = net->edge[last_num][i];
        net->edge[i][del_num] = net->edge[i][last_num];
        net->edge[last_num][i] = 0;
        net->edge[i][last_num] = 0;
        
        
        if(net->cedge[del_num][i] == 1){
            net->cedge[del_num][i] = 0;
            net->cedge[i][del_num] = 0;
        }
        
        net->cedge[del_num][i] = net->cedge[last_num][i];
        net->cedge[i][del_num] = net->cedge[i][last_num];
        net->cedge[last_num][i] = 0;
        net->cedge[i][last_num] = 0;
        
        if(net->nedge[del_num][i] == 1){
            net->nedge[del_num][i] = 0;
            net->nedge[i][del_num] = 0;
        }
        
        net->nedge[del_num][i] = net->nedge[last_num][i];
        net->nedge[i][del_num] = net->nedge[i][last_num];
        net->nedge[last_num][i] = 0;
        net->nedge[i][last_num] = 0;
    }
    
    net->node_n--;

	for(int i=0;i<del_ct;i++){
		node_delete(net, del_list[i]);
	}
}

void node_add_gng(struct gng* net)
{
	int i;
	double max_err[2];
	double min_u;
	int delete_list[GNGN];
	int delete_num = 0;
	int u;
	int q;
	int f;
	int r;

	//最大積算誤差を持つノードの決定
	max_err[0] = net->gng_err[0];
	q = 0;
	min_u = net->gng_u[0];
	u = 0;
	double min_err = net->gng_err[0];
	for (i = 1; i < net->node_n; i++) {
		if (max_err[0] < net->gng_err[i]) {
			max_err[0] = net->gng_err[i];
			q = i;
		}

		if (min_u > net->gng_u[i]) {
			min_u = net->gng_u[i];
			u = i;
		}

		if(net->gng_u[i]*1000000.0 < 100.0){
			delete_list[delete_num++] = i;
		}

		if (min_err > net->gng_err[i]) {
			min_err = net->gng_err[i];
		}
	}

	//ノードqと隣接関係にあるノードの中で最も積算誤差が大きいノードの決定
	max_err[1] = -1.0;
	f = GNGN;
	for (i = 0; i < net->node_n; i++) {
		if (net->edge[q][i] == 1 && q != i) {
			if (net->gng_err[i] > max_err[1]) {
				max_err[1] = net->gng_err[i];
				f = i;
			}
		}
	}

	//ノードrの追加
	r = net->node_n;
	for (i = 0; i < DIM; i++)
		net->node[r][i] = 0.5 * (net->node[q][i] + net->node[f][i]);

	net->edge[q][f] = 0;
	net->edge[f][q] = 0;

	net->cedge[q][f] = 0;
	net->cedge[f][q] = 0;

	net->nedge[q][f] = 0;
	net->nedge[f][q] = 0;

	net->age[q][f] = 0;
	net->age[f][q] = 0;


	net->edge[q][r] = 1;
	net->edge[r][q] = 1;

	double dis = 0.0;
	for (i = 3; i < LDIM; i++) {
		dis += (net->node[q][i] - net->node[r][i]) * (net->node[q][i] - net->node[r][i]);
	}

	if (dis < net->cthv * net->cthv) {
		net->cedge[q][r] = 1;
		net->cedge[r][q] = 1;
	}

	dis = 0.0;
	for (i = 4; i < 7; i++) {
		dis += net->node[q][i] * net->node[r][i];
	}

	if (fabs(dis) > net->nthv) {
		net->nedge[q][r] = 1;
		net->nedge[r][q] = 1;
	}

	net->age[q][r] = 0;
	net->age[r][q] = 0;

	net->edge[r][f] = 1;
	net->edge[f][r] = 1;
	net->age[r][f] = 0;
	net->age[f][r] = 0;

	dis = 0.0;
	for (i = 3; i < LDIM; i++) {
		dis += (net->node[f][i] - net->node[r][i]) * (net->node[f][i] - net->node[r][i]);
	}

	if (dis < net->cthv * net->cthv) {
		net->cedge[f][r] = 1;
		net->cedge[r][f] = 1;
	}

	dis = 0.0;
	for (i = 4; i < 7; i++) {
		dis += net->node[f][i] * net->node[r][i];
	}

	if (fabs(dis) > net->nthv) {
		net->nedge[f][r] = 1;
		net->nedge[r][f] = 1;
	}

	net->gng_err[q] -= 0.5 * net->gng_err[q];
	net->gng_err[f] -= 0.5 * net->gng_err[f];

	net->gng_u[q] -= 0.5 * net->gng_u[q];
	net->gng_u[f] -= 0.5 * net->gng_u[f];

	net->gng_err[r] = net->gng_err[q];
	net->gng_u[r] = net->gng_u[q];

	net->edge_ct[r] = 2;

	//net->disu[r] = net->disu[q];

	net->node_n++;

	//if (net->node_n > 10 && min_err < THV){
	//	//printf("Min u is %f\n", min_u*1000.0*1000.0);
	//	//node_delete(net, u);
	//	 for(int i=0;i<delete_num;i++){
	//		if(delete_list[i] > net->node_n-2) break;
	//	 	node_delete(net, delete_list[i]);
	//	 }
	//}
}

int delete_node_gngu(struct gng* net)
{
	int i;
	double min_u;
	int u;
	int flag = 0;

	//最大積算誤差を持つノードの決定
	min_u = net->gng_u[0];
	u = 0;
	double min_err = net->gng_err[0];
	for (i = 1; i < net->node_n; i++) {
		if (min_u > net->gng_u[i]) {
			min_u = net->gng_u[i];
			u = i;
		}

		if (min_err > net->gng_err[i]) {
			min_err = net->gng_err[i];
		}
	}

	if (net->node_n > 10 && min_err < THV) {
		
		node_delete(net, u);
		flag = 1;
	}

	return flag;
}

void gng_learn(struct gng* net, int s1, int s2, double* v, double e1, double e2, int flag)
{
	int i, j;
	const int MAX_AGE = 88;
	int ect = 0;
	double s_ele[10000][3];
	double cog[3];
	double r;
	double dis1;
	double ev1[6];

	double vx_s1[100], vy_s1[100];
	double pos_s1[2];
	int ct_s1 = 0;
	int cct_s1 = 0;
	double vx_s2[100], vy_s2[100];
	double pos_s2[2];
	int ct_s2 = 0;
	int cct_s2 = 0;
	double ang[100];
	double vx[100], vy[100];
	double pose[2];

	//第1勝者，第2勝者ノードに結合関係がない場合は，エッジを追加する
	if (net->edge[s1][s2] == 0) {
		net->edge[s1][s2] = 1;
		net->edge[s2][s1] = 1;
		net->edge_ct[s1]++;
		net->edge_ct[s2]++;
	}

	double dis = 0.0;
	for (i = 3; i < LDIM; i++) {
		dis += (net->node[s1][i] - net->node[s2][i]) * (net->node[s1][i] - net->node[s2][i]);
	}
	
	if (dis < net->cthv * net->cthv) {
		net->cedge[s1][s2] = 1;
		net->cedge[s2][s1] = 1;
	}
	else {
		net->cedge[s2][s1] = 0;
		net->cedge[s1][s2] = 0;
	}

	dis = 0.0;
	for (i = 4; i < 7; i++) {
		dis += net->node[s1][i] * net->node[s2][i];
	}

	for (int c = 0; c < 3; c++) {
		cog[c] = net->node[s1][c];
		s_ele[ect][c] = net->node[s1][c];
	}
	ect = 1;

	//第1勝者，第2勝者ノードの結合関係の年齢を0にする
	net->age[s1][s2] = 0;
	net->age[s2][s1] = 0;


	//第1勝者，第2勝者ノードの更新を行う
	for (i = 0; i < LDIM; i++) {
		net->node[s1][i] += e1 * (v[i] - net->node[s1][i]);
	}

	//第1勝者ノードと隣接関係を持つノードの更新を行う
	for (i = 0; i < net->node_n; i++) {
		if (net->edge[s1][i] == 1 && s1 != i) {
			for (j = 0; j < 3; j++)
				net->node[i][j] += e2 * (v[j] - net->node[i][j]);

			net->age[s1][i]++;
			net->age[i][s1]++;

			for (int c = 0; c < 3; c++) {
				s_ele[ect][c] = net->node[i][c];
				cog[c] += net->node[i][c];
			}
			ect++;

			//年齢が閾値を越えた場合は隣接関係を削除する
			if (net->age[s1][i] > MAX_AGE) {
				net->age[s1][i] = 0;
				net->age[i][s1] = 0;
				net->edge[s1][i] = 0;
				net->edge[i][s1] = 0;

				net->edge_ct[s1]--;
				net->edge_ct[i]--;

				net->cedge[s1][i] = 0;
				net->cedge[i][s1] = 0;

				net->nedge[s1][i] = 0;
				net->nedge[i][s1] = 0;

				if (net->edge_ct[i] == 0) {
					node_delete(net, i);
				}
			}
		}

		if (net->cedge[s1][i] == 1 && s1 != i)
			for (j = 3; j < LDIM; j++)
				net->node[i][j] += e2 * (v[j] - net->node[i][j]);

		if (flag != 0) {
			net->gng_err[i] -= net->dise[i] * net->gng_err[i];
			net->gng_u[i] -= net->dise[i] * net->gng_u[i];
			if (net->gng_err[i] < 0)
				net->gng_err[i] = 0.0;

			if (net->gng_u[i] < 0)
				net->gng_u[i] = 0.0;
		}
	}

	if (ect > 1) {
		for (j = 0; j < 3; j++)
			cog[j] /= (double)ect;

		r = pca(s_ele, cog, ect, ev1);

		if (ev1[1] < 0) {
			for (j = 0; j < 3; j++)
				ev1[j] *= -1.0;
		}

		dis1 = (double)invSqrt((float)(ev1[0] * ev1[0] + ev1[1] * ev1[1] + ev1[2] * ev1[2]));

		for (j = 0; j < 3; j++) {
			net->node[s1][j + 4] = ev1[j] * dis1;
		}
		net->node[s1][7] = r;
	}
	else {
		for (j = 0; j < 3; j++) {
			ev1[j] = -10.0;
			net->node[s1][j + 4] = 0.0;
			net->node[s1][7] = -10.0;
		}	
	}	
	net->node[s1][8] = ev1[3];
	net->node[s1][9] = ev1[4];
	net->node[s1][10] = ev1[5];
	
	if (fabs(dis) > net->nthv) {
		net->nedge[s1][s2] = 1;
		net->nedge[s2][s1] = 1;
	}
	else {
		net->nedge[s2][s1] = 0;
		net->nedge[s1][s2] = 0;
	}
}

#define DIS_THV 0.5
void add_new_node_distance(struct gng* net, double* v) {
	int r = net->node_n;
	int q = net->node_n + 1;

	for (int i = 0; i < 3; i++) {
		net->node[r][i] = v[i];
		net->node[q][i] = v[i] + rnd() * DIS_THV / 10.0;
	}

	for (int i = 0; i < net->node_n + 2; i++) {
		net->edge[q][i] = 0;
		net->edge[i][q] = 0;
		net->edge[r][i] = 0;
		net->edge[i][r] = 0;

		net->cedge[q][i] = 0;
		net->cedge[i][q] = 0;
		net->cedge[r][i] = 0;
		net->cedge[i][r] = 0;

		net->nedge[q][i] = 0;
		net->nedge[i][q] = 0;
		net->nedge[r][i] = 0;
		net->nedge[i][r] = 0;

		net->age[q][i] = 0;
		net->age[i][q] = 0;
		net->age[r][i] = 0;
		net->age[i][r] = 0;
	}

	net->edge[r][q] = 1;
	net->edge[q][r] = 1;
	net->cedge[r][q] = 0;
	net->cedge[q][r] = 0;
	net->nedge[r][q] = 0;
	net->nedge[q][r] = 0;
	net->edge_ct[r] = 1;
	net->edge_ct[q] = 1;

	net->node_n += 2;
}

double learn_epoch(struct gng* net, double** v, int dmax, double e1, double e2, int flag)
{
	int i, j;
	int s1, s2;
	double mindis, mindis2, * dis;
	int t;

	dis = (double*)malloc(sizeof(double) * net->node_n);
	while ((t = (int)((double)(dmax)*rnd())) == dmax);    //入力データの選択
	//第1勝者ノードの決定

	mindis = 0;

	dis[0] = 0;
	for (i = 0; i < 3; i++)
		dis[0] += (net->node[0][i] - v[t][i]) * (net->node[0][i] - v[t][i]);
	dis[1] = 0;
	for (i = 0; i < 3; i++)
		dis[1] += (net->node[1][i] - v[t][i]) * (net->node[1][i] - v[t][i]);

	if (dis[0] < dis[1]) {
		mindis = dis[0];
		s1 = 0;
		mindis2 = dis[1];
		s2 = 1;
	}
	else {
		mindis = dis[1];
		s1 = 1;
		mindis2 = dis[0];
		s2 = 0;
	}

	for (i = 2; i < net->node_n; i++) {
		dis[i] = 0;
		for (j = 0; j < 3; j++)
			dis[i] += (net->node[i][j] - v[t][j]) * (net->node[i][j] - v[t][j]);

		if (dis[i] < mindis) {
			mindis2 = mindis;
			s2 = s1;
			mindis = dis[i];
			s1 = i;
		}
		else if (dis[i] < mindis2) {
			mindis2 = dis[i];
			s2 = i;
		}
	}

	free(dis);

	if (mindis > DIS_THV * DIS_THV && net->node_n < GNGN - 2) {
		add_new_node_distance(net, v[t]);
		discount_err_gng(net);
		return 0;
	}

		//積算誤差の計算
	net->gng_err[s1] += mindis;
	//
	net->gng_u[s1] += mindis2 - mindis;
	//ノードの更新
	gng_learn(net, s1, s2, v[t], e1, e2, flag);

	/*if (net->node_n > 2 && flag == 2){
		delete_node_gngu(net);
	}*/

	return mindis;
}

int gng_main(struct gng* net, double** v, int dmax)
{
	const int ramda = 200;//def200
	int t = 0;
	double total_error = 0.0;

	for (int i1 = 0; i1 < ramda; i1++) {
		if (i1 != ramda/2)
			total_error += learn_epoch(net, v, dmax, 0.05, 0.0005, 1);
		else
			total_error += learn_epoch(net, v, dmax, 0.05, 0.0005, 2);
	}

	//ノードの追加
	total_error /= (double)ramda;
	if (net->node_n < GNGN){
		node_add_gng(net);
	}

	return t;
}
