#include "rnd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <random>
#include <time.h>

// 一様乱数 (0.0 <= x <= 1.0)
double rnd() /* uniform random number */
{
	return((double)(rand() % 30001) / 30000.0);
}

// 正規分布に従う疑似乱数 (-6.0 <= x <= 6.0)
double rndn() /* normal random number */
{
	return (rnd() + rnd() + rnd() + rnd() + rnd() + rnd() +
		rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0);
}

// 毎回シードを変更する最大値と最小値の範囲内の値を出力する疑似乱数
int random(const int min, const int max) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(min, max);
	return dis(gen);
}