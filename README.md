# 2023年TI杯全国大学生电子设计竞赛控制E题

## 题目简介：运动目标控制与自动追踪系统

本项目为2023年TI杯控制E题的嵌入式平台代码部分，四天三夜写出来的代码非常简陋，但是最终还是完成了全部指标，也拿到了江苏省省一。
视觉部分代码主要是对黑色边框A4纸张的识别，发送四个坐标点到stm32进行处理，最关键的是坐标变换和舵机运动方向。
（代码仅供参考）

### 主要功能模块

1. **运动目标控制系统**：通过红色激光笔模拟运动目标，光斑落在距离1m的白色屏幕上
2. **自动追踪系统**：绿色激光笔跟踪红色光斑，实现自动追踪功能
3. **图像识别处理**：OpenMV视觉识别模块，实现矩形框检测和激光点定位
4. **精确云台控制**：双轴舵机云台，实现激光笔精确定位

## 系统架构图

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   OpenMV视觉    │    │   STM32主控      │    │   舵机云台      │
│   识别模块       │───→│   控制系统      │───→ │   驱动模块      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ↓                       ↓                       ↓
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  激光点识别     │    │  PID控制算法     │     │  激光笔定位     │
│  矩形框检测     │    │  轨迹规划        │     │  角度控制       │
│  坐标提取       │    │  串口通信        │     │  PWM输出        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 代码框架结构

### 1. 主控制层（USER/main.c）
```c
// 系统初始化流程
void SystemInit() {
    // 硬件初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);
    LED_Init();
    OLED_Init();

    // PWM初始化（舵机控制）
    TIM1_PWM_Init(20000-1, 168-1);   // PE13
    TIM9_PWM_Init(20000-1, 168-1);   // PE5

    // 通信初始化
    Serial_Init();      // 与OpenMV通信
    Serial2_Init();
    Serial3_Init();

    // 定时器中断初始化（5ms控制周期）
    TIM3_Int_Init(1000-1, 1680-1);
}

// 主控制循环
int main(void) {
    SystemInit();

    // PID参数初始化
    Kp_X = 9; Kp_Y = 9;
    Ki_X = 0.3; Ki_Y = 0.3;
    Kd_X = 0.5; Kd_Y = 0.5;

    // 系统复位到中心位置
    sevro_angle(220, 995);

    while(1) {
        // 主循环中处理用户输入和模式切换
        HandleModeSwitch();
        UpdateOLEDDisplay();
    }
}
```

### 2. 中断控制层（TIMER/timer.c）
```c
// 5ms定时中断处理函数
void TIM3_IRQHandler(void) {
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        switch(mode) {
            case 1: // 红点复位模式
                TrackingToCenter();
                break;

            case 2: // 铅笔框模式（固定矩形）
                RectangleTrackingMode();
                break;

            case 3: // 自由矩形模式
                FreeRectangleMode();
                break;

            case 4: // OpenMV检测矩形模式
                OpenMVRectangleMode();
                break;

            default: // 默认跟踪模式
                DefaultTrackingMode();
                break;
        }
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
```

### 3. PID控制算法层（PID/pid.c）
```c
// 增量式PID控制算法实现
float PID_X(float target_val, float actual_val) {
    target_val_X = target_val;
    err_X = target_val_X - actual_val;
    integral_X += err_X;

    // 增量式PID公式
    output_val_X = Kp_X * (err_X - err_last1_X) +
                   Ki_X * err_X +
                   Kd_X * (err_X - err_last1_X + err_last2_X - err_last1_X);

    err_last2_X = err_last1_X;
    err_last1_X = err_X;

    return output_val_X;
}

float PID_Y(float target_val, float actual_val) {
    // Y轴PID控制实现（同X轴）
    target_val_Y = target_val;
    err_Y = target_val_Y - actual_val;
    integral_Y += err_Y;

    output_val_Y = Kp_Y * (err_Y - err_last1_Y) +
                   Ki_Y * err_Y +
                   Kd_Y * (err_Y - err_last1_Y + err_last2_Y - err_last1_Y);

    err_last2_Y = err_last1_Y;
    err_last1_Y = err_Y;

    return output_val_Y;
}
```

### 4. 舵机控制层（MOTOR/motor.c）
```c
// 舵机角度控制函数
void sevro_angle(float angleX, float angleY) {
    // 将角度转换为PWM占空比
    TIM_SetCompare1(TIM9, angleX / 180 * 20 + 1500);
    TIM_SetCompare2(TIM9, angleY / 180 * 20 + 1500);
}

// 轨迹规划与点到点移动
void servo_degress_points_to_move(float zero[2], int points[4][2], int step) {
    if (point_n == 4) {
        // 最后一个点，计算到第一个点的过渡
        taget[0] = (points[0][0] - points[3][0]) / step * step_n + points[3][0];
        taget[1] = (points[0][1] - points[3][1]) / step * step_n + points[3][1];
    } else if(point_n == 0) {
        // 从中心点到第一个顶点
        taget[0] = (points[point_n][0] - zero[0]) / step * step_n + zero[0];
        taget[1] = (points[point_n][1] - zero[1]) / step * step_n + zero[1];
    } else {
        // 顶点间移动
        taget[0] = (points[point_n][0] - points[point_n-1][0]) / step * step_n + points[point_n-1][0];
        taget[1] = (points[point_n][1] - points[point_n-1][1]) / step * step_n + points[point_n-1][1];
    }

    // 步进计数
    step_n++;
    if (step_n > step) {
        step_n = 0;
        point_n++;
        point_c++;
        if (point_n > 4) {
            point_n = 1;
        }
    }
}
```

### 5. 串口通信层（Serial/Serial.c）
```c
// OpenMV数据接收与解析
void USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 接收OpenMV发送的坐标数据
        uint8_t received_data = USART_ReceiveData(USART1);

        // 数据解析协议
        if(received_data == 0xFF) {
            // 数据帧头标识
            data_index = 0;
            receive_flag = 1;
        } else if(receive_flag) {
            openmv_data[data_index++] = received_data;

            if(data_index >= DATA_LENGTH) {
                // 数据接收完成，解析坐标
                ParseCoordinateData();
                receive_flag = 0;
                data_index = 0;
            }
        }
    }
}

// 坐标数据解析
void ParseCoordinateData(void) {
    // 解析红色激光点坐标
    Red_Coordinates[0] = (openmv_data[0] << 8) | openmv_data[1];
    Red_Coordinates[1] = (openmv_data[2] << 8) | openmv_data[3];

    // 解析矩形顶点坐标
    for(int i = 0; i < 4; i++) {
        Rectangle_Coordinates[i][0] = (openmv_data[4 + i*4] << 8) | openmv_data[5 + i*4];
        Rectangle_Coordinates[i][1] = (openmv_data[6 + i*4] << 8) | openmv_data[7 + i*4];
    }
}
```

## 核心算法实现

### 1. 坐标系建立与标定
```c
// 建立图像坐标系到云台角度的映射关系，通过位置摆放确定
#define zero_x 78.5    // 图像中心X坐标
#define zero_y 55.5    // 图像中心Y坐标

// 云台角度零点（对应图像中心）
float zero_Ang[2] = {197, 995};

// 坐标变换函数
void CoordinateTransform(float img_x, float img_y, float* angle_x, float* angle_y) {
    // 图像坐标到云台角度的线性映射
    *angle_x = zero_Ang[0] + K_x * (img_x - zero_x);
    *angle_y = zero_Ang[1] + K_y * (img_y - zero_y);
}
```

### 2. 轨迹规划算法
```c
// 矩形轨迹规划
void RectangleTrajectoryPlanning(int points[4][2], int steps) {
    // 计算矩形四个顶点间的直线轨迹
    for(int i = 0; i < 4; i++) {
        int next_point = (i + 1) % 4;

        // 计算该边的步进增量
        float dx = (points[next_point][0] - points[i][0]) / (float)steps;
        float dy = (points[next_point][1] - points[i][1]) / (float)steps;

        // 生成中间点坐标
        for(int j = 0; j < steps; j++) {
            trajectory[i*steps + j][0] = points[i][0] + dx * j;
            trajectory[i*steps + j][1] = points[i][1] + dy * j;
        }
    }
}
```

### 3. PID参数自适应调节
```c
// 根据不同模式调节PID参数
void AdaptPIDParameters(int mode) {
    switch(mode) {
        case 2: // 铅笔框模式 - 需要精确跟踪
            Kp_X = 8; Kp_Y = 8;
            Ki_X = 0.9; Ki_Y = 0.9;
            Kd_X = 0.1; Kd_Y = 0.1;
            break;

        case 3: // 自由矩形模式 - 平衡精度与稳定性
            Kp_X = 10; Kp_Y = 10;
            Ki_X = 0.28; Ki_Y = 0.28;
            Kd_X = 0.05; Kd_Y = 0.05;
            break;

        case 4: // OpenMV检测模式 - 快速响应
            Kp_X = 10; Kp_Y = 10;
            Ki_X = 0.3; Ki_Y = 0.3;
            Kd_X = 0.1; Kd_Y = 0.1;
            break;

        default: // 默认跟踪模式
            Kp_X = 9; Kp_Y = 9;
            Ki_X = 0.3; Ki_Y = 0.3;
            Kd_X = 0.5; Kd_Y = 0.5;
            break;
    }
}
```

### 4. 闭环控制算法
```c
// 主控制算法流程
void MainControlLoop(void) {
    // 1. 获取目标坐标（轨迹规划或用户输入）
    GetTargetPosition(&target_x, &target_y);

    // 2. 获取当前激光点实际位置（OpenMV反馈）
    GetCurrentPosition(&current_x, &current_y);

    // 3. PID控制计算
    float control_x = PID_X(target_x, current_x);
    float control_y = PID_Y(target_y, current_y);

    // 4. 角度限幅与输出
    Ang_x -= control_x;
    Ang_y -= control_y;

    // 5. 角度限制
    if(Ang_x < 0) Ang_x = 0;
    if(Ang_x > 360) Ang_x = 360;
    if(Ang_y < 0) Ang_y = 0;
    if(Ang_y > 1800) Ang_y = 1800;

    // 6. 舵机角度输出
    sevro_angle(Ang_x, Ang_y);
}
```

## 硬件配置

### 主要器件清单
- **主控MCU**: STM32F407ZGZGT6
- **视觉模块**: OpenMV H7 Plus
- **执行机构**: 高精度数字舵机 × 2
- **激光模块**: 红色/绿色激光笔
- **显示模块**: OLED显示屏
- **通信接口**: UART串口通信

### 引脚配置
```c
// 舵机PWM控制引脚
TIM1: PE13 (舵机1 - X轴)
TIM9: PE5  (舵机2 - Y轴)

// 串口通信引脚
USART1: PA9(TX), PA10(RX) - OpenMV通信
USART2: 调试串口
USART3: 备用通信

// LED指示灯
LED0: PF9
LED1: PF10

// 按键输入
KEY0: PE4
KEY1: PE3
KEY2: PE2
```

## 算法优化策略

### 1. 控制稳定性优化
- **增量式PID**：避免积分饱和，提高系统稳定性
- **参数自适应**：根据不同任务模式动态调节PID参数
- **死区处理**：设置控制死区，减少抖动

### 2. 实时性优化
- **中断驱动**：5ms定时中断确保控制实时性
- **数据缓存**：使用环形缓冲区处理串口数据
- **算法简化**：采用快速计算方法减少运算时间

### 3. 精度提升策略
- **坐标标定**：建立精确的图像坐标到云台角度映射
- **轨迹平滑**：采用分段线性插值平滑轨迹
- **反馈校正**：基于视觉反馈的闭环校正



