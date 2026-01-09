/**
 *******************************************************************************
 * @file           : tsk_chassis.cpp
 * @brief          : 舵轮底盘控制任务
 *******************************************************************************
 * @attention
 *
 * 本文件实现舵轮底盘的 FreeRTOS 任务逻辑，主要负责：
 * - 周期性运行底盘控制循环
 * - 读取测试输入或遥控指令
 * - 调用底盘控制器更新舵轮状态
 *
 * 该任务作为系统调度层的一部分，不包含具体运动学或底层驱动实现。
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "CubeMX_BSP/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "CubeMX_BSP/Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "para_chassis.hpp"
#include <cstdint>
#include <cmath>
/* TODO: 等 crt_chassis.hpp 完成后替换为正式头文件 */
// #include "crt_chassis.hpp"

/* 临时底盘接口（等待人员A完成正式接口） */
class TemporaryCrtChassis {
private:
    bool m_initialized;
    
public:
    TemporaryCrtChassis() : m_initialized(false) {}
    
    /* 模拟初始化函数 */
    void init() {
        m_initialized = true;
    }
    
    void setCmd(float vx, float vy, float wz) {
        if (!m_initialized) return;
        // 空实现，等待正式接口
    }
    
    void update(float dt) {
        if (!m_initialized) return;
        // 空实现，等待正式接口
    }
      /* 检查是否初始化成功 */
    bool isReady() const {
        return m_initialized;
    }
};

/* Typedef -------------------------------------------------------------------*/
typedef enum {
    MODE_NORMAL = 0,
    MODE_SMALL_GYRO,
    MODE_STOP,
    MODE_COUNT
} ChassisMode_t;

/* 错误标志位定义 */
typedef enum {
    ERROR_NONE = 0,
    ERROR_CHASSIS_NOT_READY = (1 << 0),
    ERROR_INVALID_MODE = (1 << 1),
} ChassisError_t;

/* Define --------------------------------------------------------------------*/
#define CHASSIS_TASK_PERIOD_MS  1
#define CHASSIS_TASK_STACK_SIZE 512
#define CHASSIS_TASK_PRIORITY   5
/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
static class TemporaryCrtChassis {
private:
    bool m_initialized;
    
public:
    TemporaryCrtChassis() : m_initialized(false) {}
    
    void init() { m_initialized = true; }
    
    void setCmd(float vx, float vy, float wz) {
        if (!m_initialized) return;
        // 空实现，等待正式接口
    }
    
    void update(float dt) {
        if (!m_initialized) return;
        // 空实现，等待正式接口
    }
    bool isReady() const {
        return m_initialized;
    }
} g_chassis;

static ChassisMode_t g_currentMode = MODE_NORMAL;// 默认停止状态
static uint32_t g_errorFlags = ERROR_NONE;
static bool g_initialized = false;

/* Function prototypes -------------------------------------------------------*/
static void vChassisTask(void *pvParameters);
static void setChassisModeInternal(ChassisMode_t mode);
static void updateErrorFlags(void);


/* User code -----------------------------------------------------------------*/
/**
  * @brief  内部模式设置函数
  * @param  mode: 目标模式
  * @note   模式切换时的特殊处理应在此函数中实现
  */
static void setChassisModeInternal(ChassisMode_t mode) {
    if (mode >= MODE_COUNT) {
        g_errorFlags |= ERROR_INVALID_MODE;
        return;
    }
    
    ChassisMode_t prevMode = g_currentMode;
    g_currentMode = mode;
    
    /* 模式切换时的特殊处理 */
    if (mode == MODE_STOP) {
        // 立即发送停止指令，确保安全
        g_chassis.setCmd(0.0f, 0.0f, 0.0f);
    }
    
    /* TODO: 根据实际需求添加模式切换处理
     * 例如：
     * 1. NORMAL -> SMALL_GYRO: 需要平滑过渡
     * 2. SMALL_GYRO -> NORMAL: 需要停止自旋
     */
}

/**
  * @brief  更新错误标志
  */
static void updateErrorFlags(void) {
    g_errorFlags = ERROR_NONE;
    
    if (!g_chassis.isReady()) {
        g_errorFlags |= ERROR_CHASSIS_NOT_READY;
    }
    
    if (g_currentMode >= MODE_COUNT) {
        g_errorFlags |= ERROR_INVALID_MODE;
    }
}

/**
  * @brief  获取速度指令
  * @param  vx: 输出X方向速度
  * @param  vy: 输出Y方向速度
  * @param  wz: 输出角速度
  * @note   根据当前模式生成速度指令
  * 
  * 速度指令来源：
  * 1. NORMAL模式：应从遥控器/上位机获取（TODO：等待接口）
  * 2. SMALL_GYRO模式：固定角速度自旋
  * 3. STOP模式：零速
  */
static void getVelocityCommand(float &vx, float &vy, float &wz) {
    switch (g_currentMode) {
        case MODE_NORMAL:
            /* TODO: 此处应从遥控器/决策层获取真实速度指令
             * 当前使用测试值，等待实际接口
             */
            vx = 0.5f;  // 测试值
            vy = 0.0f;  // 测试值
            wz = 0.5f;  // 测试值
            break;
            
        case MODE_SMALL_GYRO:
            /* 小陀螺模式：原地旋转 */
            vx = 0.0f;
            vy = 0.0f;
            wz = 3.14f;  // π rad/s ≈ 180°/s
            break;
            
        case MODE_STOP:
        default:
            /* 停止模式或无效模式：零速 */
            vx = 0.0f;
            vy = 0.0f;
            wz = 0.0f;
            break;
    }
}

/**
  * @brief  底盘任务主函数
  */
static void vChassisTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS);
    
    // 初始化底盘
    g_chassis.init();
    g_initialized = true;
    
    // 测试：模式切换计数器
    uint32_t modeTestCounter = 0;
    const uint32_t MODE_TEST_INTERVAL = 3000;  // 3秒切换一次模式
    
    for (;;) {
        // 更新错误标志
        updateErrorFlags();
        
        // 检查底盘是否就绪
        if (!g_chassis.isReady()) {
            // 底盘未就绪，等待下一个周期
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue;
        }
        
        // 测试：自动切换模式（仅用于调试，实际应由外部控制）
        modeTestCounter += CHASSIS_TASK_PERIOD_MS;
        if (modeTestCounter >= MODE_TEST_INTERVAL) {
            modeTestCounter = 0;
            ChassisMode_t nextMode = static_cast<ChassisMode_t>(
                (g_currentMode + 1) % MODE_COUNT
            );
            setChassisModeInternal(nextMode);
        }
        
        // 获取速度指令
        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        getVelocityCommand(vx, vy, wz);
        
        /* TODO: 此处可添加指令滤波或限幅
         * 例如：
         * 1. 加速度限制
         * 2. 速度限幅
         * 3. 指令平滑
         */
        
        // 调用底盘控制接口
        g_chassis.setCmd(vx, vy, wz);
        g_chassis.update(CHASSIS_TASK_PERIOD_MS / 1000.0f);
        
        /* 等待下一个周期 */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
  * @brief  底盘任务初始化
  */
void tskChassisInit(void) {
    xTaskCreate(vChassisTask,
                "ChassisTask",
                CHASSIS_TASK_STACK_SIZE,
                NULL,
                CHASSIS_TASK_PRIORITY,
                NULL);
}

/* C语言兼容接口 */
extern "C" {
    void TSK_CHASSIS_Init(void) {
        tskChassisInit();
    }
    
    /* 最小化的外部接口 */
    void TSK_CHASSIS_SetMode(uint8_t mode) {
        if (mode < MODE_COUNT) {
            setChassisModeInternal(static_cast<ChassisMode_t>(mode));
        }
    }
    
    uint8_t TSK_CHASSIS_GetMode(void) {
        return static_cast<uint8_t>(g_currentMode);
    }
    
    uint32_t TSK_CHASSIS_GetError(void) {
        return g_errorFlags;
    }
}