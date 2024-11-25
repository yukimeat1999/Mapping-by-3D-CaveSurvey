/*
 *  gng.c
 *  Claster
 *
 *  Created by Yuichiro Toda on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */

#include "gng.h"

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
	for (i = 0; i < GNGN; i++) {
		for(int j=0;j<LDIM;j++){
			net->node[i][j] = 0.0;
		}

		for(int j=LDIM;j<DIM;j++){
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
