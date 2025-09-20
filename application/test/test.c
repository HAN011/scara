// app
#include "kalman_filter.h"
#include "motor_def.h"
#include "general_def.h"
// module
#include "dji_motor.h"
#include "servo_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

#include "test.h"
#include "usermode.hpp"
#include <math.h>
#include <stdint.h>
#include <string.h>

#define POS_X       100
#define POS_Y       100
#define ERROR_LIMIT 3

typedef struct {
    DJIMotorInstance *motor;
    float radio;
    float angle;
    float original_angle;
    float target;
    float len;
} _Joint;
static _Joint joint[4];
static ServoInstance *jaw;
static uint8_t *usb;
typedef struct {
    float x;
    float y;
    float z;
} _coord;
_coord target_coord = {
    .x = 358,
    .y = 0,
    .z = 0};
_coord offset = {
    .x = 0, //-41.26,
    .y = 0, // 52.85,
    .z = 0};
static _coord pos                  = {0};
static uint8_t stop_flag           = 1;
static uint8_t jaw_clamp_flag      = 1;
static uint8_t jaw_clamp_last_flag = 1;
static uint8_t is_clamp_turn       = 0;
typedef struct {
    float x;
    float y;
    float z;
    uint8_t is_clamp_turn;
    uint8_t jaw_clamp_flag;
} _MOVE;
_MOVE example_1[] = {
    {172, 0, 120, 0, 0},

    {172, 0, 100, 0, 1},
    {-60, 160, -20 + 1, 0, 0},

    {172, 0, 80, 0, 1},
    {-121, 160, -20 + 1, 0, 0},

    {172, 0, 60, 0, 1},
    {-182, 160, -20 + 1, 0, 0},

    {172, 0, 40, 0, 1},
    {-243, 160, -20 + 1, 0, 0},

    {172, 0, 20, 0, 1},
    {-60, 160, 0 + 1, 0, 0},

    {172, 0, 0, 0, 1},
    {-121, 160, 0 + 1, 0, 0},

    {172, 0, -20, 0, 1},
    {-182, 160, 0 + 1, 0, 0},

    //---
    {232, 0, 100, 0, 1},
    {-243, 160, 0 + 1, 0, 0},

    {232, 0, 80, 0, 1},
    {-60, 160, 20 + 1, 0, 0},

    {232, 0, 60, 0, 1},
    {-121, 160, 20 + 1, 0, 0},

    {232, 0, 40, 0, 1},
    {-182, 160, 20 + 1, 0, 0},

    {232, 0, 20, 0, 1},
    {-243, 160, 20 + 1, 0, 0},

    {232, 0, 0, 0, 1},
    {-60, 160, 40 + 1, 0, 0},

    {232, 0, -20, 0, 1},
    {-121, 160, 40 + 1, 0, 0},

    //---
    {292, 0, 100, 0, 1},
    {-182, 160, 40 + 1, 0, 0},

    {292, 0, 80, 0, 1},
    {-243, 160, 40 + 1, 0, 0},

    {292, 0, 60, 0, 1},
    {-60, 160, 60 + 1, 0, 0},

    {292, 0, 40, 0, 1},
    {-121, 160, 60 + 1, 0, 0},

    {292, 0, 20, 0, 1},
    {-182, 160, 60 + 1, 0, 0},

    {292, 0, 0, 0, 1},
    {-243, 160, 60 + 1, 0, 0},

    {292, 0, -20, 0, 1},
    {-60, 160, 80 + 1, 0, 0},

    //---
    {352, 0, 100, 0, 1},
    {-121, 160, 80 + 1, 0, 0},

    {352, 0, 80, 0, 1},
    {-182, 160, 80 + 1, 0, 0},

    {352, 0, 60, 0, 1},
    {-243, 160, 80 + 1, 0, 0},

    {352, 0, 40, 0, 1},
    {-60, 160, 100 + 1, 0, 0},

    {352, 0, 20, 0, 1},
    {-121, 160, 100 + 1, 0, 0},

    {352, 0, 0, 0, 1},
    {-182, 160, 100 + 1, 0, 0},

    {352, 0, -20, 0, 1},
    {-243, 160, 100 + 1, 0, 0},
};
int example_1_len = sizeof(example_1) / sizeof(_MOVE);

_MOVE example_2[] = {
    {172, 0, 120, 0, 0},

    {172, 0, 100, 0, 1},
    {-60, 160, -20 + 2, 0, 0},

    {172, 0, 80, 0, 1},
    {-121, 160, -20 + 2, 0, 0},

    {172, 0, 60, 0, 1},
    {-182, 160, -20 + 2, 0, 0},

    {172, 0, 40, 0, 1},
    {-243, 160, -20 + 2, 0, 0},

    {172, 0, 20, 0, 1},
    {-30, 220, -20 + 2, 1, 0},

    {172, 0, 0, 0, 1},
    {-30, 281, -20 + 2, 1, 0},

    {172, 0, -20, 0, 1},
    {-30, 342, -20 + 2, 1, 0},

    //---
    {232, 0, 100, 0, 1},
    {-60, 160, 0 + 2, 0, 0},

    {232, 0, 80, 0, 1},
    {-121, 160, 0 + 2, 0, 0},

    {232, 0, 60, 0, 1},
    {-182, 160, 0 + 2, 0, 0},

    {232, 0, 40, 0, 1},
    {-243, 160, 0 + 2, 0, 0},

    {232, 0, 20, 0, 1},
    {-30, 220, 0 + 2, 1, 0},

    {232, 0, 0, 0, 1},
    {-30, 281, 0 + 2, 1, 0},

    {232, 0, -20, 0, 1},
    {-30, 342, 0 + 2, 1, 0},

    //---
    {292, 0, 100, 0, 1},
    {-60, 160, 20 + 2, 0, 0},

    {292, 0, 80, 0, 1},
    {-121, 160, 20 + 2, 0, 0},

    {292, 0, 60, 0, 1},
    {-182, 160, 20 + 2, 0, 0},

    {292, 0, 40, 0, 1},
    {-243, 160, 20 + 2, 0, 0},

    {292, 0, 20, 0, 1},
    {-30, 220, 20 + 2, 1, 0},

    {292, 0, 0, 0, 1},
    {-30, 281, 20 + 2, 1, 0},

    {292, 0, -20, 0, 1},
    {-30, 342, 20 + 2, 1, 0},

    //---
    {352, 0, 100, 0, 1},
    {-60, 160, 40 + 2, 0, 0},

    {352, 0, 80, 0, 1},
    {-121, 160, 40 + 2, 0, 0},

    {352, 0, 60, 0, 1},
    {-182, 160, 40 + 2, 0, 0},

    {352, 0, 40, 0, 1},
    {-243, 160, 40 + 2, 0, 0},

    {352, 0, 20, 0, 1},
    {-30, 220, 40 + 2, 1, 0},

    {352, 0, 0, 0, 1},
    {-30, 281, 40 + 2, 1, 0},

    {352, 0, -20, 0, 1},
    {-30, 342, 40 + 2, 1, 0},
};
int example_2_len = sizeof(example_2) / sizeof(_MOVE);

_MOVE example_3[] = {
    {172, 0, 120, 0, 0},

    {172, 0, 100, 0, 1},
    {-60, 160, -20 + 1, 0, 0},

    {172, 0, 80, 0, 1},
    {-121, 160, -20 + 1, 0, 0},

    {172, 0, 60, 0, 1},
    {-182, 160, -20 + 1, 0, 0},

    {172, 0, 40, 0, 1},
    {-243, 160, -20 + 1, 0, 0},

    {172, 0, 20, 0, 1},
    {-60, 160, 0 + 1, 0, 0},

    {172, 0, 0, 0, 1},
    {-121, 160, 0 + 1, 0, 0},

    {172, 0, -20, 0, 1},
    {-182, 160, 0 + 1, 0, 0},

    //---
    {232, 0, 100, 0, 1},
    {-243, 160, 0 + 1, 0, 0},

    {232, 0, 80, 0, 1},
    {-60, 160, 20 + 1, 0, 0},

    {232, 0, 60, 0, 1},
    {-121, 160, 20 + 1, 0, 0},

    {232, 0, 40, 0, 1},
    {-182, 160, 20 + 1, 0, 0},

    {232, 0, 20, 0, 1},
    {-243, 160, 20 + 1, 0, 0},

    {232, 0, 0, 0, 1},
    {-60, 160, 40 + 1, 0, 0},

    {232, 0, -20, 0, 1},
    {-121, 160, 40 + 1, 0, 0},

    //---
    {292, 0, 100, 0, 1},
    {-182, 160, 40 + 1, 0, 0},

    {292, 0, 80, 0, 1},
    {-243, 160, 40 + 1, 0, 0},

    {292, 0, 60, 0, 1},
    {-60, 160, 60 + 1, 0, 0},

    {292, 0, 40, 0, 1},
    {-121, 160, 60 + 1, 0, 0},

    {292, 0, 20, 0, 1},
    {-182, 160, 60 + 1, 0, 0},

    {292, 0, 0, 0, 1},
    {-243, 160, 60 + 1, 0, 0},

    {292, 0, -20, 0, 1},
    {-60, 160, 80 + 1, 0, 0},

    //---
    {352, 0, 100, 0, 1},
    {-121, 160, 80 + 1, 0, 0},

    {352, 0, 80, 0, 1},
    {-182, 160, 80 + 1, 0, 0},

    {352, 0, 60, 0, 1},
    {-243, 160, 80 + 1, 0, 0},

    {352, 0, 40, 0, 1},
    {-60, 160, 100 + 1, 0, 0},

    {352, 0, 20, 0, 1},
    {-121, 160, 100 + 1, 0, 0},

    {352, 0, 0, 0, 1},
    {-182, 160, 100 + 1, 0, 0},

    {352, 0, -20, 0, 1},
    {-243, 160, 100 + 1, 0, 0},
};
int example_3_len = sizeof(example_3) / sizeof(_MOVE);

uint8_t current_state = 0;
/*
state_mode_map = {
    "0": none
    "1": 上位机控制
    "2": 自动模式-内容1-正执行
    "3": 自动模式-内容1-逆执行
    "4": 自动模式-内容2-正执行
    "5": 自动模式-内容2-逆执行
    "6": 自动模式-内容3-正执行
    "7": 自动模式-内容3-逆执行
}
*/
void USBRxEventCallback(uint16_t len)
{
    uint8_t buf[255];
    memcpy(buf, usb, len);
    jaw_clamp_last_flag = jaw_clamp_flag;
    if (buf[0] == '1') {
        sscanf((char *)buf, "1 x:%f y:%f z:%f stop_flag:%hhu is_clamp_turn:%hhu jaw_clamp_flag:%hhu",
               &target_coord.x, &target_coord.y, &target_coord.z, &stop_flag, &is_clamp_turn, &jaw_clamp_flag);
        current_state = 0;
    } else if (buf[0] == '2') {
        current_state = 2;
        stop_flag     = 0;
    } else if (buf[0] == '3') {
        current_state = 3;
        stop_flag     = 0;
    } else if (buf[0] == '4') {
        stop_flag     = 0;
        current_state = 4;
    } else if (buf[0] == '5') {
        stop_flag     = 0;
        current_state = 5;
    } else if (buf[0] == '6') {
        stop_flag     = 0;
        current_state = 6;
    } else if (buf[0] == '7') {
        stop_flag     = 0;
        current_state = 7;
    } else if (buf[0] == '8') {
        stop_flag     = 0;
        current_state = 8;
    } else if (buf[0] == '9') {
        stop_flag     = 0;
        current_state = 9;
    }
}

void TestInit()
{
    Motor_Init_Config_s motor1_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 20,    // 4.5
                .Ki            = 0,     // 1
                .Kd            = 0.001, // 3
                .IntegralLimit = 160,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 40000,
            },
            .speed_PID = {
                .Kp            = 3,    // 4.5
                .Ki            = 0,    // 1
                .Kd            = 0.01, // 3
                .IntegralLimit = 4000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 8000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M2006,
    };

    Motor_Init_Config_s motor2_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 9, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 160,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .speed_PID = {
                .Kp            = 3, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 1000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s motor3_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 9, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 160,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 18000,
            },
            .speed_PID = {
                .Kp            = 3, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 4000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s motor4_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 5, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 160,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            .speed_PID = {
                .Kp            = 3, // 4.5
                .Ki            = 0, // 1
                .Kd            = 0, // 3
                .IntegralLimit = 4000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M2006,
    };

    joint[0].motor = DJIMotorInit(&motor1_config);
    joint[0].radio = -0.002715568139392416f;
    joint[0].len   = 0;

    joint[1].motor = DJIMotorInit(&motor2_config);
    joint[1].radio = 0.0120196641705831f;
    joint[1].len   = 207.9975422931723f;
    joint[2].motor = DJIMotorInit(&motor3_config);
    joint[2].radio = 0.0136798905608755f;
    joint[2].len   = 150.0031066344961f;
    joint[3].motor = DJIMotorInit(&motor4_config);
    joint[3].radio = -0.0284450063211125f;
    joint[3].len   = 0;

    Servo_Init_Config_s servo_config = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim    = &htim1,
        .Channel = TIM_CHANNEL_1,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Free_Angle_mode,
        .Servo_type       = Servo180,
    };
    jaw = ServoInit(&servo_config);

    USB_Init_Config_s usb_conf = {
        .rx_cbk = USBRxEventCallback,
    };
    usb = USBInit(usb_conf);
}

static int example_index = 0;
void handle_example()
{
    static uint8_t last_state = 0;
    static uint8_t step_index = 0;

    static _MOVE *example;
    static int example_len;
    static int reversed_flag = 0;

    if (current_state != last_state) {
        step_index = 0;
        if (current_state == 2) {
            example       = example_1;
            example_len   = example_1_len;
            example_index = 0;
            reversed_flag = 0;
        } else if (current_state == 3) {
            example       = example_1;
            example_len   = example_1_len;
            example_index = example_1_len - 1;
            reversed_flag = 1;
        } else if (current_state == 4) {
            example       = example_2;
            example_len   = example_2_len;
            example_index = 0;
            reversed_flag = 0;
        } else if (current_state == 5) {
            example       = example_2;
            example_len   = example_2_len;
            example_index = example_2_len - 1;
            reversed_flag = 1;
        } else if (current_state == 6) {
            example       = example_3;
            example_len   = example_3_len;
            example_index = 0;
            reversed_flag = 0;
        } else if (current_state == 7) {
            example       = example_3;
            example_len   = example_3_len;
            example_index = example_3_len - 1;
            reversed_flag = 1;
        }
        target_coord.x = pos.x;
        target_coord.y = pos.y;
        target_coord.z = pos.z;
    }
    last_state = current_state;

    if (current_state == 0) {
        return;
    }

    _MOVE e = example[example_index];

    static int a = 0;
    switch (step_index) {
        case 0:
            int safe_height = 128;
            if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == safe_height) {
                step_index++;
            }

            target_coord.z = safe_height;

            break;
        case 1:
            if (fabs(joint[1].target - joint[1].angle) < 1 && fabs(joint[2].target - joint[2].angle) < 1 && fabs(joint[3].target - joint[3].angle) < 1 && target_coord.x == e.x && target_coord.y == e.y)
                step_index++;
            target_coord.x = e.x;
            target_coord.y = e.y;
            is_clamp_turn  = e.is_clamp_turn;
            break;
        case 2:
            if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == e.z) {
                step_index++;
            }
            target_coord.z = e.z;
            break;
        case 3:
            if (reversed_flag)
                jaw_clamp_flag = !e.jaw_clamp_flag;
            else
                jaw_clamp_flag = e.jaw_clamp_flag;

            a++;
            if (a > 1000) {
                step_index = 0;
                if (reversed_flag) {
                    example_index--;
                    if (example_index < 0)
                        current_state = 0;
                } else {
                    example_index++;
                    if (example_index == example_len)
                        current_state = 0;
                }
                a = 0;
            }
            break;
    }
}
typedef struct {
    _MOVE place[50];
    int place_len;
    _MOVE fetch[50];
    int fetch_len;
} _MOVE_V2;
// _MOVE_V2 example1_v2 = {
//     .place = {
//         {}
//     }
// }

// void handle_example_v2()
// {
//     static uint8_t last_state = 0;
//     static uint8_t step_index = 0;

//     static _MOVE_V2 *example;
//     static int reversed_flag = 0;

//     if (current_state != last_state) {
//         step_index = 0;
//         if (current_state == 2) {
//             example       = example1_v2;
//             example_index = 0;
//             reversed_flag = 0;
//         }
//         target_coord.x = pos.x;
//         target_coord.y = pos.y;
//         target_coord.z = pos.z;
//     }
//     last_state = current_state;

//     if (current_state == 0) {
//         return;
//     }

//     _MOVE e = example[example_index];

//     switch (step_index) {
//         case 0:
//             int safe_height = 120;
//             if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == safe_height) {
//                 step_index++;
//             }

//             target_coord.z = safe_height;

//             break;
//         case 1:
//             if (fabs(joint[1].target - joint[1].angle) < 1 && fabs(joint[2].target - joint[2].angle) < 1 && fabs(joint[3].target - joint[3].angle) < 1 && target_coord.x == e.x && target_coord.y == e.y)
//                 step_index++;
//             target_coord.x = e.x;
//             target_coord.y = e.y;
//             is_clamp_turn  = e.is_clamp_turn;
//             break;
//         case 2:
//             if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == e.z) {
//                 step_index++;
//             }
//             target_coord.z = e.z;
//             break;
//         case 3:
//             if (reversed_flag)
//                 jaw_clamp_flag = !e.jaw_clamp_flag;
//             else
//                 jaw_clamp_flag = e.jaw_clamp_flag;

//             static int a = 0;
//             a++;
//             if (a > 1000) {
//                 step_index++;
//                 if (reversed_flag) {
//                     example_index--;
//                     if (example_index < 0)
//                         current_state = 0;
//                 } else {
//                     example_index++;
//                     if (example_index == example_len)
//                         current_state = 0;
//                 }
//                 a = 0;
//             }
//             break;
//         case 4:
//             int safe_height = 120;
//             if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == safe_height) {
//                 step_index++;
//             }

//             target_coord.z = safe_height;

//             break;
//         case 5:
//             if (fabs(joint[1].target - joint[1].angle) < 1 && fabs(joint[2].target - joint[2].angle) < 1 && fabs(joint[3].target - joint[3].angle) < 1 && target_coord.x == e.x && target_coord.y == e.y)
//                 step_index++;
//             target_coord.x = e.x;
//             target_coord.y = e.y;
//             is_clamp_turn  = e.is_clamp_turn;
//             break;
//         case 6:
//             if (fabs(joint[0].target - joint[0].angle) < 1 && target_coord.z == e.z) {
//                 step_index++;
//             }
//             target_coord.z = e.z;
//             break;
//         case 7:
//             if (reversed_flag)
//                 jaw_clamp_flag = !e.jaw_clamp_flag;
//             else
//                 jaw_clamp_flag = e.jaw_clamp_flag;

//             static int a = 0;
//             a++;
//             if (a > 1000) {
//                 step_index++;
//                 if (reversed_flag) {
//                     example_index--;
//                     if (example_index < 0)
//                         current_state = 0;
//                 } else {
//                     example_index++;
//                     if (example_index == example_len)
//                         current_state = 0;
//                 }
//                 a = 0;
//             }
//             break;
//     }
// }

/* 测试任务 */
void _TestTask()
{
    static int key_mode     = 2;
    static float sign_speed = 10000;
    switch (key_mode) {
        case 1:
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
                joint[0].motor->motor_settings.outer_loop_type = SPEED_LOOP;
                joint[0].motor->measure.total_round            = 0;
                sign_speed++;
                if (sign_speed > 50000) sign_speed = 50000;
                DJIMotorSetRef(joint[0].motor, sign_speed);
                DJIMotorEnable(joint[0].motor);
                DJIMotorEnable(joint[1].motor);
                DJIMotorEnable(joint[2].motor);
                DJIMotorEnable(joint[3].motor);
                return;
            } else {
                sign_speed                                     = 10000;
                joint[0].motor->motor_settings.outer_loop_type = ANGLE_LOOP;
            }
            break;
        case 2:
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
                current_state = 2;
            }
    }

    for (int i = 0; i < 4; i++) {
        if (stop_flag) {
            joint[i].motor->stop_flag = 0;
        } else {
            joint[i].motor->stop_flag = 1;
        }
        joint[i].original_angle = joint[i].motor->measure.total_angle;
        joint[i].angle          = joint[i].original_angle * joint[i].radio;
    }

    handle_example();

    static float debug_d = 0;
    if (debug_d) {
        joint[0].motor->measure.total_round += debug_d / joint[0].radio / 360;
        debug_d = 0;
    }
    // 二轴scara逆解
    float x = target_coord.x - offset.x;
    float y = target_coord.y - offset.y;
    float z = target_coord.z - offset.z;

    float l1 = joint[1].len;
    float l2 = joint[2].len;
    float l  = sqrtf(powf(x, 2) + powf(y, 2));

    float a1 = acosf((powf(l1, 2) + powf(l, 2) - powf(l2, 2)) / (2 * l1 * l));
    float a2 = acosf((powf(l1, 2) + powf(l2, 2) - powf(l, 2)) / (2 * l1 * l2));
    float a3 = acosf((powf(l2, 2) + powf(l, 2) - powf(l1, 2)) / (2 * l2 * l));

    float j1_angle;
    if (x > 0)
        j1_angle = RAD_2_DEGREE * (atanf(y / x) - a1);
    else
        j1_angle = RAD_2_DEGREE * (atanf(y / x) - a1) + 180;
    float j2_angle = (180 - RAD_2_DEGREE * a2); // a2-pi-j1
    float j3_angle = -(a3 - fabs(atanf(y / x))) * RAD_2_DEGREE;

    // 设定关节角度值
    joint[0].target = z;
    joint[1].target = j1_angle;
    joint[2].target = j2_angle;
    joint[3].target = j3_angle;

    // 正运动学解算
    pos.x = cosf(DEGREE_2_RAD * joint[1].angle) * joint[1].len + cosf(DEGREE_2_RAD * joint[2].angle + DEGREE_2_RAD * joint[1].angle) * joint[2].len;
    pos.y = sinf(DEGREE_2_RAD * joint[1].angle) * joint[1].len + sinf(DEGREE_2_RAD * joint[2].angle + DEGREE_2_RAD * joint[1].angle) * joint[2].len;
    pos.z = joint[0].angle;

    if (is_clamp_turn)
        joint[3].target += 90;

    for (int i = 0; i < 4; i++) {
        DJIMotorSetRef(joint[i].motor, joint[i].target / joint[i].radio);
    }

    // 舵机
    static int16_t jaw_clamp_angle = 159;
    // static int16_t jaw_loosen_angle  = 145;
    static int16_t jaw_loosen_angle = 120;
    if (stop_flag) {
        Servo_Motor_FreeAngle_Set(jaw, 159);
    } else {
        if (jaw_clamp_flag) {
            Servo_Motor_FreeAngle_Set(jaw, jaw_clamp_angle);
        } else if (jaw_loosen_angle) {
            Servo_Motor_FreeAngle_Set(jaw, jaw_loosen_angle);
        }
    }
}
