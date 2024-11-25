#include "rnd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <random>
#include <time.h>

// ��l���� (0.0 <= x <= 1.0)
double rnd() /* uniform random number */
{
	return((double)(rand() % 30001) / 30000.0);
}

// ���K���z�ɏ]���^������ (-6.0 <= x <= 6.0)
double rndn() /* normal random number */
{
	return (rnd() + rnd() + rnd() + rnd() + rnd() + rnd() +
		rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0);
}

// ����V�[�h��ύX����ő�l�ƍŏ��l�͈͓̔��̒l���o�͂���^������
int random(const int min, const int max) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(min, max);
	return dis(gen);
}