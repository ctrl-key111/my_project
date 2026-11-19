//
// Created by DQJ on 2026/11/18.
//
#include "stm32h7xx_hal.h"
#include "Music.h"
#include <stdint.h>
#include "tim.h"
#include "stm32h7xx_hal_tim.h"
#include "User/middlewares/devices/key/key.h"
//音乐怎么不算是一种算法呢
#define P0 	0	// 休止符频率

#define L1 262  // 低音频率
#define L2 294
#define L3 330
#define L4 349
#define L5 392
#define L6 440
#define L7 494

#define M1 523  // 中音频率
#define M2 587
#define M3 659
#define M4 698
#define M5 784
#define M6 880
#define M7 988

#define H1 1047 // 高音频率
#define H2 1175
#define H3 1319
#define H4 1397
#define H5 1568
#define H6 1760
#define H7 1976

// 节拍速度(每分钟多少拍)
#define BPM 132
//TIM12频率
uint32_t timFrequency =1000000u;

// 播放进度
uint32_t playIndex = 0;

// 每拍的持续时间
float noteDuration = (60000.0f) / BPM;

const Bate MoChouXiang[] = {
    // 我被困在了
    {M6, 1}, {M5, 1}, {M3, 1}, {M5, 0.5f}, {M5, 0.5f},
    // 这片混沌 柳暗
    {M6, 0.5f}, {M5, 1}, {M5, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M3, 0.5f},
    // 花明 一村一村一村
    {M2, 0.5f}, {M3, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M5, 0.5f}, {M3, 0.5f},
    // 一村又一村
    {M2, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {L7, 0.5f}, {M3, 1}, {M1, 1},
    // 不能理顺我
    {M2, 1}, {M3, 1}, {M2, 1}, {M3, 0.5f}, {M3, 0.5f},

    // ......

    // 娃儿抬头望
    {M3, 0.5f}, {M2, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M2, 1.5f},
    // 姥姥在天上
    {M1, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M2, 1}, {M1, 1},
  };


const Bate haojiubujian[] = {
    // 前奏
    {M1, 0.5f}, {M7, 0.5f}, {M1, 0.5f}, {M5, 0.5f}, {M1, 0.5f}, {M5, 0.5f}, {M2, 0.5f}, {M1, 0.5f},
    {M1, 0.5f}, {M7, 0.5f}, {M1, 0.5f}, {M7, 0.5f}, {M1, 0.5f}, {M5, 0.5f}, {M2, 0.5f}, {M1, 0.5f},
    {M1, 0.5f}, {M6, 0.5f}, {M1, 0.5f}, {M6, 0.5f}, {M1, 0.5f}, {M5, 0.5f}, {M2, 0.5f}, {M1, 0.5f},
    {M2, 2.0f}, {M7, 2.0f},

    // 我来到 你的城市
    {M5, 1.0f}, {M4, 1.0f}, {M3, 1.0f}, {0, 1.0f},
    {M7, 1.0f}, {M2, 1.0f}, {M7, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 0.5f},

    // 走过你来时的路
    {0, 0.5f}, {M1, 0.5f}, {M6, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M1, 1.0f},
    {M2, 2.0f}, {0, 1.0f}, {0, 1.0f},

    // 想象着 没我的日子 你是怎样 的孤独
    {0, 0.5f}, {M5, 0.5f}, {M4, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {0, 1.0f},
    {M1, 0.5f}, {M1, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 0.5f}, {M5, 0.5f}, {M5, 0.5f},
    {M6, 1.0f}, {M4, 1.0f}, {M3, 1.0f}, {M5, 1.0f},
    {M2, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M2, 1.5f}, {0, 1.0f},

    // 拿着你 给的照片 熟悉的那一条街 只是没了
    {0, 0.5f}, {M5, 0.5f}, {M4, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {0, 1.0f},
    {M7, 0.5f}, {M2, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 1.0f},
    {0, 0.5f}, {M5, 0.5f}, {M6, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M1, 1.0f},
    {M2, 2.0f}, {0, 1.0f}, {0, 1.0f},

    // 熟悉的那一条街 只是没了
    {0, 0.5f}, {M5, 0.5f}, {M4, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M4, 1.0f},
    {M5, 0.5f}, {M4, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 0.5f}, {M5, 0.5f},
    {M4, 0.5f}, {M3, 0.5f}, {M2, 1.0f}, {0, 1.0f}, {0, 1.0f},

    // 你的画面 我们回不到 那天
    {M5, 1.0f}, {M1, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 0.5f}, {M6, 0.5f}, {M7, 0.5f},
    {M1, 0.5f}, {M3, 0.5f}, {M6, 0.5f}, {M6, 0.5f}, {M2, 0.5f}, {M1, 1.0f},
    {M1, 2.0f}, {0, 1.0f}, {0, 0.5f}, {M3, 0.5f},

    // 你会不会忽然的出现 在街角的咖啡店
    {M5, 1.0f}, {M3, 0.5f}, {M3, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M5, 1.0f},
    {M3, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 1.0f}, {0, 1.0f},
    {M5, 1.0f}, {M3, 0.5f}, {M3, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M5, 1.0f},
    {M3, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 1.0f}, {0, 1.0f},

    // 我多么想和你见一面 看看你最近改变
    {M2, 0.5f}, {M1, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M6, 1.0f}, {0, 1.0f},
    {M6, 0.5f}, {M5, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M3, 1.0f}, {0, 0.5f}, {M3, 0.5f},
    {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {M6, 0.5f}, {0, 0.5f}, {0, 0.5f}, {M1, 0.5f}, {M2, 0.5f},

    // 不去说从前 只是寒暄 对你说一句
    {M2, 0.5f}, {M1, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M6, 1.0f}, {0, 1.0f},
    {M6, 0.5f}, {M5, 0.5f}, {M2, 0.5f}, {M3, 0.5f}, {M3, 1.0f}, {0, 0.5f}, {M3, 0.5f},
    {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {M6, 0.5f}, {0, 0.5f}, {0, 0.5f}, {M1, 0.5f}, {M2, 0.5f},

    // 只是说一句 好久不见
    {M3, 0.5f}, {M5, 0.5f}, {M3, 0.5f}, {M2, 0.5f}, {M1, 0.5f}, {M1, 0.5f}, {0, 0.5f}, {0, 0.5f}, {M3, 0.5f}, {M6, 0.5f}, {M2, 1.0f},
    {M2, 0.5f}, {M1, 2.5f}, {0, 1.0f},

    // 尾奏
    {M1, 1.0f}, {M2, 1.0f}, {M3, 1.0f}, {M5, 1.0f},
    {M2, 1.0f}, {M3, 1.0f}, {M1, 1.0f}, {M1, 1.0f},
    {M3, 1.0f}, {M5, 1.0f}, {M1, 2.0f},
    {M2, 0.5f}, {M3, 0.5f}, {M3, 0.5f}, {M4, 0.5f}, {M5, 2.0f},
    {M5, 4.0f}
};


void music_start() {
    key_set();

        if (State)
        {
            const Bate bate = haojiubujian[playIndex];
            if (bate.frequency == P0)
            {
                // 休止符
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            }
            else
            {
                // 将频率转换为计数值, 设置到自动重装载寄存器
                uint32_t arr = timFrequency / bate.frequency;
                __HAL_TIM_SET_AUTORELOAD(&htim12,arr);
                // 设置占空比为20%
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, arr / 5); // 20%占空比
                // 从0开始计数 重置PWM波形
                __HAL_TIM_SET_COUNTER(&htim12, 0);
            }
            // 延时该音符的持续时间 (5ms的空白以区分连续两个相同的音符)
            HAL_Delay((uint32_t) (bate.period * noteDuration) - 5);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            HAL_Delay(5);

            // 下一个音符
            playIndex++;
            // 播放结束
            if (playIndex >= sizeof(MoChouXiang) / sizeof(MoChouXiang[0]))
            {
                State = 0;
                playIndex = 0;
            }
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
        }


}
