//
// Created by DQJ on 2026/11/19.
//

#ifndef DMC02_MYTEST_VOFA_H
#define DMC02_MYTEST_VOFA_H
#include <stdint.h>

#define VOFA_JUSTFLOAT_MAX_CH   10u
#define VOFA_BUF_SIZE           ((VOFA_JUSTFLOAT_MAX_CH + 1u) * 4u)

void Vofa_JustFloat(const float *data, uint8_t num);

#endif //DMC02_MYTEST_VOFA_H
