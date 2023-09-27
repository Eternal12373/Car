#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_tockn.h>

#define led_on() digitalWrite(2, LOW)
#define led_off() digitalWrite(2, HIGH)

/*---------config---------*/
//#define STOP_DEBUG
#define MPU6050_LOOP

class PIDController
{
private:
    float kp, ki, kd;
    float integral, previousError;
    float integralLimit, outputLimit;

public:
    PIDController(float kp, float ki, float kd, float integralLimit, float outputLimit)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->integralLimit = integralLimit;
        this->outputLimit = outputLimit;
        this->integral = 0;
        this->previousError = 0;
    }

    float compute(float input, float setpoint)
    {
        float error = setpoint - input;
        integral += error;
        // Add integral windup prevention
        if (integral > integralLimit)
            integral = integralLimit;
        if (integral < -integralLimit)
            integral = -integralLimit;

        float derivative = error - previousError;
        previousError = error;

        float output = kp * error + ki * integral + kd * derivative;

        // Limit the output
        if (output > outputLimit)
            output = outputLimit;
        if (output < -outputLimit)
            output = -outputLimit;

        return output;
    }
    void pid_reset()
    {
        this->integral = 0;
        this->previousError = 0;
    }
};

enum
{
    INIT_STATE, // 识别，串口通信，保存地图，
    FOLLOW_LINE,
    TURN_STATE,
    SLOW_STRAIGHT_BEFORE_TURN,
    DETECT_STATE,
    STRIKE_STATE,
    TURN_180,
    DELAY_FOLLOW_LINE,
    STATE_COUNT,
};

const char *stateNames[STATE_COUNT] =
    {
        "INIT",
        "FOLLOW",
        "TURN",
        "SLOW",
        "DETECT",
        "STRIKE",
        "TURN180",
        "DELAY",
};
uint8_t trace_num = 15;
uint8_t last_trace_num = 15;

int robot_state = INIT_STATE;
float yaw;
float yaw_target;

int is_real_flag = 0;
int is_stop_flag = 0;

int is_start_detect = 0;

int straight_speed = 100;

unsigned long start_time = 0;
unsigned long current_time = 0;
unsigned long stop_start_time = 0;
unsigned long turn90_start_time = 0;
unsigned long turn180_start_time = 0;
unsigned long strike_start_time = 0;
unsigned long delay_follow_start_time = 0;

unsigned long strike_current_time = 0;

PIDController rotate_yaw(40, 0.5, 0, 400, 1300);

MPU6050 mpu6050(Wire);

/**
 * @brief 巡线传感器引脚定义
 *
 */
#define IO_X1 36 // 左
#define IO_X2 39
#define IO_X3 34
#define IO_X4 35
#define IO_X5 23 // 右
/**
 * @brief 电机接口定义
 *
 */
#define IO_M1PWM 32
#define IO_M2PWM 18
#define IO_M3PWM 33
#define IO_M4PWM 19
#define IO_M1IN1 14
#define IO_M1IN2 13
#define IO_M2IN1 17
#define IO_M2IN2 5
#define IO_M3IN1 26
#define IO_M3IN2 27
#define IO_M4IN1 16
#define IO_M4IN2 4
/**
 * @brief 超声波接口定义
 */
#define IO_TRIG 12
#define IO_ECHO 25

/*-----OLED Definition-------*/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char *sta_ssid = "LunarLobster";
const char *sta_password = "12345678";

#define MOTOR_DISTANCE_TO_CENTER 0.1

/*--------Keyboard Definition--------*/
int index1;
int index2;
int index3;
int index4;
int index5;
int index6;
int index7;

String joy_x;
String joy_y;
String but_a;
String but_b;
String but_x;
String but_y;
String but_l;
String but_r;

uint8_t send_buff[20];
uint8_t rx_buff[128] = {0};

WiFiUDP Udp;

int open_flag = 0;

/*------函数定义--------*/
/**
 * @brief 电机引脚初始化
 */
void MotorInit(void);
/**
 * @brief 电机速度设置
 */
void SetDirectionAndSpeed(int speed1, int speed2, int speed3, int speed4);
void SetSpeed(float vx_set, float vy_set, float wz_set);
/**
 * @brief 超声波引脚初始化
 */
void UltrasonicInit(void);
/**
 * @brief 超声波测距
 *
 * @return int
 */
// int UltrasonicDistence(void);
/**
 * @brief 灰度巡线模块数据 x1 x2 x3 x4
 *
 * @return uint8_t
 */
uint8_t GetLine(void);
/**
 * 陀螺仪
 */

void get_mpu6050(float *angleX, float *angleY, float *angleZ);
void tracing(void);

void display_mode(int mode);

float limit(float value, float min_value, float max_value);
void start_detect(void);

long duration;
float distance;
int rx_length = 0;
uint8_t rx_char = 0;
/*
  1
4   2
  3
*/
// uint8_t the_way_arr[128] = {0};
// uint8_t the_way_arr[128] = {4, 4, 2, 2, 4, 2, 1, 2, 4, 2, 1, 4, 1, 1, 4, 3, 2, 1, 2, 2, 4, 4, 2, 4, 3, 4, 1, 4, 2, 2, 3, 2, 4, 4, 2, 3, 4, 2, 2, 1, 4, 2, 1, 1, 4, 4, 2, 3, 4, 2, 2, 1, 4, 1, 1, 4, 3, 1, 2, 4, 2, 4, 2, 4, 4, 1, 4, 3, 2, 1, 1, 2, 4, 1, 2, 1, 4, 2, 4, 4, 3, 2, 2, 2, 4, 2, 4, 4, 2, 2}; // 判断方向的数组
//   uint8_t the_way_arr[128] = {4, 3, 2, 2, 4, 2, 1, 2, 4, 2, 1, 4, 1, 1, 4, 3, 2, 1, 2, 2, 4, 4, 2, 4, 3, 4, 1, 4, 2, 2, 3, 2, 4, 4, 2, 3, 4, 2, 2, 1, 4, 2, 1, 1, 4, 4, 2, 3, 4, 2, 2, 1, 4, 1, 1, 4, 3, 1, 2, 4, 2, 4, 2, 4, 4, 1, 4, 3, 2, 1, 1, 2, 4, 1, 2, 1, 4, 2, 4, 4, 3, 2, 2, 2, 4, 2, 4, 4, 2, 2}; // 判断方向的数组
uint8_t the_way_arr[128] = {4, 4, 2, 2, 4, 2, 4, 2, 1, 2, 2, 3, 4, 1, 2, 4, 4, 2, 1, 4, 2, 2, 2, 3, 4, 4, 2, 2, 4, 1, 4, 3, 4, 4, 3, 2, 1, 2, 2, 4, 2, 4, 2, 4, 1, 3, 1, 2, 4, 2, 2, 2, 3, 4, 2, 2, 4, 3, 4, 1, 2, 4, 4, 2, 1, 4, 2, 2, 2, 3, 4, 4, 2, 2, 4, 1, 1, 4, 3, 2, 2, 3, 2, 2, 2, 4, 1, 2, 3, 4, 4, 4, 2, 4, 1, 3, 2, 2, 2, 4, 2, 4, 4, 2, 2};
int cross_cnt = 0; // 经过路口的数量，索引

int rx_cnt = 0;
int rx_finish = 0;
int err_time = 0;
int err_flag = 0;
void serialEvent()
{
    // Serial.println("serialEvent");
    // Serial.println(Serial.available());
    err_time = 0;
    if (robot_state == INIT_STATE)
    {
        while (Serial.available())
        {
            uint8_t incomingChar = Serial.read();
            rx_char = incomingChar;
            if (rx_cnt != 0)
            {
                if (incomingChar == 'b')
                {
                    // Serial.println("rx_finish");
                    rx_finish = 1;
                    rx_length = rx_cnt - 1;
                    rx_cnt = 0;
                    // for(int i=0;i<rx_length;i++)
                    // {
                    //     Serial.printf("%d/n",rx_buff[i]);
                    // }
                    break;
                }
                the_way_arr[rx_cnt - 1] = incomingChar;
                rx_cnt++;
            }
            if (rx_cnt == 0)
            {
                if (incomingChar == 'a')
                {
                    rx_cnt++;
                }
            }
        }
    }
    else
    {
        while (Serial.available())
        {
            rx_char = Serial.read();

            // if (robot_state != DETECT_STATE)
            // {
            //     if (rx_char == 2 && the_way_arr[cross_cnt] == 3)
            //     {
            //         is_stop_flag++;
            //     }
            // }

            // if (robot_state == DETECT_STATE)
            if (the_way_arr[cross_cnt] == 3)
            {
                if (rx_char == 1)
                {
                    is_real_flag = 1;
                }
            }
        }
    }
}

void setup()
{
    pinMode(2, OUTPUT);
    Serial.begin(9600);
    Wire.begin();
    delay(1000);
#if 1
    delay(500);

    /**各种外设初始化**/
    MotorInit();
    LineInit();
    UltrasonicInit();

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        for (;;)
            ;
    }
    display.display();
    delay(2000);

    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    display.clearDisplay();

#endif

    display.setTextColor(SSD1306_WHITE);
    led_on();
    delay(500);
    led_off();
}

int count_flag = 0;
float dis_sum = 0.0;
void loop()
{
    delay(2);
    // if (err_time < 3000)
    // {
    //     // err_time++;
    // }
    mpu6050.update();

    yaw = mpu6050.getAngleZ();

    duration = UltrasonicDistence();
    if (count_flag <= 5)
    {
        count_flag++;
        dis_sum += duration * 0.034 / 2;
        if (count_flag == 5)
        {
            distance = dis_sum / 5;
            count_flag = 0;
            dis_sum = 0.0;
        }
    }

    // if (rx_finish)
    // {
    //     for (int i = 0; i < rx_length; i++)
    //     {
    //         display.clearDisplay();
    //         display.setCursor(0, 0);
    //         display.setTextSize(2);
    //         display.printf("i:%d\n", i);
    //         display.printf("%d", the_way_arr[i]);
    //         display.display();
    //         delay(1000);
    //     }
    //     rx_finish = 0;
    // }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    // display.printf("yaw: %f\n", yaw);
    // display.printf("err: %f\n", yaw - yaw_target);
    // display.printf("cnt: %d\n", cross_cnt);
    // display.printf("tx:%d\n", is_start_detect);
    // display.printf("is_real:%d\n", is_real_flag);
    // display.printf("is_stop:%d\n", is_stop_flag);
    display.printf("char:%d\n", rx_char);
    // display.printf("rx_cnt%d\n", rx_cnt);
    // display.printf("err%d\n",err_time);
    display.printf("is_finish%d\n", rx_finish);
    // display.printf("distance:%f\n", distance);
    // display.printf("turn_dir:%d\n", the_way_arr[cross_cnt]);
    // display.printf("mode:%d", GetLine());
    // display.println(stateNames[robot_state]);
    // if (err_time > 2000)
    // {
    //     display.printf("Dumped!!!\n");
    // }
    display.display();

    // 状态改变

    switch (robot_state)
    {
    case INIT_STATE:
        // led_on();
        if (GetLine() == 0b00100) // 任意一个传感器检测到黑，启动
        {
            robot_state = FOLLOW_LINE;
        }
        break;

    case FOLLOW_LINE:
        // led_off();
        if (distance <= 43 && distance >= 1 && the_way_arr[cross_cnt] == 3)
        {
            is_stop_flag = 1;
        }
        if (is_stop_flag)
        {
            led_on();
            robot_state = DETECT_STATE;
            stop_start_time = millis();
        }
        tracing();
        break;
    case DETECT_STATE:
        if (millis() - stop_start_time < 500)
        {
            SetSpeed(0, 0, 0);
        }
        else
        {
            if (is_real_flag)
            {
                strike_start_time = millis();
                robot_state = STRIKE_STATE;
            }
            else
            {
                robot_state = TURN_180;
                turn180_start_time = millis();
                yaw_target = yaw + 180;
            }
        }

        break;
    case STRIKE_STATE:
        if (millis() - strike_start_time < 700)
        {
            SetSpeed(80, 0, 0);
        }
        else if (millis() - strike_start_time < 1200)
        {
            SetSpeed(-80, 0, 0);
        }
        else
        {
            turn180_start_time = millis();
            robot_state = TURN_180;
            yaw_target = yaw + 180;
        }
        break;

    case TURN_180:
    {
        int output_180 = rotate_yaw.compute(yaw, yaw_target);
        if (millis() - turn180_start_time < 300)
        {
            SetSpeed(0, 0, 0);
        }
        else if (millis() - turn180_start_time < 800)
        {
            SetSpeed(0, 0, -output_180);
        }
        else if (fabs(yaw - yaw_target) > 5 && (((1 << 2) & GetLine()) != (1 << 2)))
        {
            SetSpeed(0, 0, -output_180);
        }
        else
        {
            cross_cnt++;
            SetSpeed(0, 0, 0);
            is_real_flag = 0;
            is_stop_flag = 0;
            robot_state = FOLLOW_LINE;
            last_trace_num = 0b00100; // 保证直行
        }
        break;
    }
    case SLOW_STRAIGHT_BEFORE_TURN:
        if (millis() - start_time < 200)
        {
            SetSpeed(0, 0, 0);
        }
        else if (millis() - start_time < 380)
        {
            // led_on();
            SetSpeed(80, 0, 0);
        }
        else
        {
            if (the_way_arr[cross_cnt] == 1)
            {
                robot_state = FOLLOW_LINE;
            }
            if (the_way_arr[cross_cnt] == 2)
            {
                yaw_target = yaw - 90;
                robot_state = TURN_STATE;
                turn90_start_time = millis();
                rotate_yaw.pid_reset();
            }
            else if (the_way_arr[cross_cnt] == 4)
            {
                yaw_target = yaw + 90;
                robot_state = TURN_STATE;
                turn90_start_time = millis();
                rotate_yaw.pid_reset();
            }
            cross_cnt++;
        }
        break;
    case DELAY_FOLLOW_LINE:
        if (millis() - delay_follow_start_time < 1000)
        {
            tracing();
        }
        else
        {
            robot_state = FOLLOW_LINE;
        }
        break;
    case TURN_STATE:
    {
        if (the_way_arr[cross_cnt] == 3)
        {
            is_stop_flag = 0;
        }
        int output = (int)(rotate_yaw.compute(yaw, yaw_target));
        if (millis() - turn90_start_time < 300)
        {
            SetSpeed(0, 0, -output);
        }
        else if (fabs(yaw - yaw_target) > 5 && (((1 << 2) & GetLine()) != (1 << 2))) //&& (((1<<2) & GetLine())!=(1<<2))
        {
            SetSpeed(0, 0, -output);
        }
        else
        {
            SetSpeed(0, 0, 0);
            last_trace_num = 0b00100; // 直行
            if (the_way_arr[cross_cnt] == 3)
            {
                robot_state = DELAY_FOLLOW_LINE;
                delay_follow_start_time = millis();
            }
            else
            {
                robot_state = FOLLOW_LINE;
            }
        }
        break;
    }
    }

    // for (int i = 0; i < 128; i++)
    // {
    //     Serial.print(rx_buff[i]);
    // }

    //    mpu6050.update();
    //   Serial.print("angleX : ");
    //   Serial.print(mpu6050.getAngleX());
    //   Serial.print("\tangleY : ");
    //   Serial.print(mpu6050.getAngleY());
    //   Serial.print("\tangleZ : ");
    //   Serial.println(mpu6050.getAngleZ());
    //  duration=UltrasonicDistence();
    //  distance= duration*0.034/2;
    //  Serial.println(distance);
    // Serial.println(GetLine());
    // SetSpeed(100,0,0);
    // SetSpeed(100,0,200);
}

/**
 * @brief 电机引脚初始化
 */
void MotorInit(void)
{
    pinMode(IO_M1PWM, OUTPUT);
    pinMode(IO_M2PWM, OUTPUT);
    pinMode(IO_M3PWM, OUTPUT);
    pinMode(IO_M4PWM, OUTPUT);

    pinMode(IO_M1IN1, OUTPUT);
    pinMode(IO_M1IN2, OUTPUT);
    pinMode(IO_M2IN1, OUTPUT);
    pinMode(IO_M2IN2, OUTPUT);
    pinMode(IO_M3IN1, OUTPUT);
    pinMode(IO_M3IN2, OUTPUT);
    pinMode(IO_M4IN1, OUTPUT);
    pinMode(IO_M4IN2, OUTPUT);
}

/**
 * @brief 电机速度设置
 */
void SetDirectionAndSpeed(int speed1, int speed2, int speed3, int speed4)
{
    /*不同电机接线方向可能不同，改IN1 和 IN2的逻辑*/
    if (speed1 < 0)
    {
        speed1 *= -1;
        digitalWrite(IO_M1IN1, HIGH);
        digitalWrite(IO_M1IN2, LOW);
        analogWrite(IO_M1PWM, speed1);
    }
    else
    {
        digitalWrite(IO_M1IN1, LOW);
        digitalWrite(IO_M1IN2, HIGH);
        analogWrite(IO_M1PWM, speed1);
    }
    if (speed2 < 0)
    {
        speed2 *= -1;
        digitalWrite(IO_M2IN1, LOW);
        digitalWrite(IO_M2IN2, HIGH);
        analogWrite(IO_M2PWM, speed2);
    }
    else
    {
        digitalWrite(IO_M2IN1, HIGH);
        digitalWrite(IO_M2IN2, LOW);
        analogWrite(IO_M2PWM, speed2);
    }
    if (speed3 < 0)
    {
        speed3 *= -1;
        digitalWrite(IO_M3IN1, HIGH);
        digitalWrite(IO_M3IN2, LOW);
        analogWrite(IO_M3PWM, speed3);
    }
    else
    {
        digitalWrite(IO_M3IN1, LOW);
        digitalWrite(IO_M3IN2, HIGH);
        analogWrite(IO_M3PWM, speed3);
    }
    if (speed4 < 0)
    {
        speed4 *= -1;
        digitalWrite(IO_M4IN2, HIGH);
        digitalWrite(IO_M4IN1, LOW);
        analogWrite(IO_M4PWM, speed4);
    }
    else
    {
        digitalWrite(IO_M4IN2, LOW);
        digitalWrite(IO_M4IN1, HIGH);
        analogWrite(IO_M4PWM, speed4);
    }
}

void UltrasonicInit(void)
{
    pinMode(IO_TRIG, OUTPUT);
    pinMode(IO_ECHO, INPUT);
}
int UltrasonicDistence(void)
{
    digitalWrite(IO_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(IO_TRIG, LOW);
    return pulseIn(IO_ECHO, HIGH);
}
void LineInit(void)
{
    pinMode(IO_X1, INPUT);
    pinMode(IO_X2, INPUT);
    pinMode(IO_X3, INPUT);
    pinMode(IO_X4, INPUT);
    pinMode(IO_X5, INPUT);
}

/**
 * 黑线为1 白线为0
 * x1 x1 x3 x4 x5是真实分布
 */
uint8_t GetLine(void)
{
    uint8_t x1, x2, x3, x4, x5;
    uint8_t tmp = 0;

    x1 = digitalRead(IO_X1);
    x2 = digitalRead(IO_X2);
    x3 = digitalRead(IO_X3);
    x4 = digitalRead(IO_X4); // 黑线为0 白线为1
    x5 = digitalRead(IO_X5);

    tmp = (x1 << 4) | (x2 << 3) | (x3 << 2) | (x4 << 1) | x5;
    return tmp;
}
void tracing(void)
{

    float current_yaw = 0.0;

    trace_num = GetLine();

    if (((1 & trace_num) == 1) || (((1 << 4) & trace_num) == (1 << 4)))
    {

        // if (the_way_arr[cross_cnt + 1] == 3) //下一次要掉头
        // {
        //     straight_speed = 33;
        // }

        robot_state = SLOW_STRAIGHT_BEFORE_TURN;
        start_time = millis();
        return;
        // display.clearDisplay();

        // display.setTextSize(2);
        // display.setTextColor(SSD1306_WHITE);
        // display.setCursor(0, 20);
        // display.println("Turning");
        // display.display();

        // SetSpeed(50, 0, 0);
        // delay(400);

        // if (the_way_arr[cross_cnt] == 1)
        // {
        //     SetSpeed(100, 0, 0);
        //     delay(200);
        // }
        // else if (the_way_arr[cross_cnt] == 2)
        // {
        //     SetSpeed(0, 0, 1000);
        //     delay(820);
        //     // while (1)
        //     // {
        //     //     SetSpeed(0, 0, 1000);
        //     //     if (((1 << 2) & GetLine()) == (1 << 2))
        //     //         break;
        //     // }
        // }
        // else if (the_way_arr[cross_cnt] == 3)
        // {
        //     SetSpeed(0, 0, 0);
        //     delay(5000);
        // }
        // else if (the_way_arr[cross_cnt] == 4)
        // {
        //     SetSpeed(0, 0, -1000);
        //     delay(820);
        //     // while (1)
        //     // {
        //     //     SetSpeed(0, 0, -1000);
        //     //     if (((1 << 2) & GetLine()) == (1 << 2))
        //     //         break;
        //     // }
        // }
        // SetSpeed(0, 0, 0);
        // delay(100);

        // cross_cnt++;
    }
    else
    {
        if (trace_num == 0)
        {
            trace_num = last_trace_num;
        }
        switch (trace_num)
        {

            // case 0b00000:
        case 0b00100:
            if (the_way_arr[cross_cnt] == 3)
                SetSpeed(35, 0, 0); // go straight
            else
                SetSpeed(98, 0, 0); // go straight 110
            break;
        case 0b00110:
        case 0b00010:
            if (the_way_arr[cross_cnt] == 3)
            {
                SetSpeed(5, 0, 500); // turn right 500
            }
            else
            {
                SetSpeed(30, 0, 380);
            }
            break;
        case 0b01100:
        case 0b01000:
            if (the_way_arr[cross_cnt] == 3)
            {
                SetSpeed(5, 0, -500); // turn left 500
            }
            else
            {
                SetSpeed(30, 0, -380);
            }
            break;
        default:
            SetSpeed(0, 0, 0);
        }
        last_trace_num = trace_num;
    }
}
/*
x
|  -
|   -
|    >
|__________y
wz_set 顺时针为正
*/
void SetSpeed(float vx_set, float vy_set, float wz_set)
{
    int speed1, speed2, speed3, speed4;
    speed1 = vx_set + vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    speed2 = vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    speed3 = vx_set - vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    speed4 = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;

#ifdef STOP_DEBUG
    speed1 = speed2 = speed3 = speed4 = 0;
#endif
    SetDirectionAndSpeed(speed1, speed2, speed3, speed4);
}

void display_mode(int mode)
{
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("mode: ");
    display.println(mode);
    display.display();
}

void get_mpu6050(float *angleX, float *angleY, float *angleZ)
{
}

float limit(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    else if (value > max_value)
    {
        return max_value;
    }
    else
    {
        return value;
    }
}

void start_detect(void)
{
    Serial.printf("%d", 1);
}