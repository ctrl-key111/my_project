//
// Created by DQJ on 2026/11/18.
//

#ifndef DMC02_MYTEST_BUZZER_H
#define DMC02_MYTEST_BUZZER_H
#include <stdint.h>


typedef struct
{
    uint16_t frequency; // 音符频率
    float period; // 音符持续时间，单位为拍
} Bate;

//音乐的启用
void music_start();

#endif //DMC02_MYTEST_BUZZER_H
