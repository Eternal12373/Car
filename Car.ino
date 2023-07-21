#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_tockn.h>

//#define STOP_DEBUG

MPU6050 mpu6050(Wire);

/**
 * @brief 巡线传感器引脚定义
 *
 */
#define IO_X1 35
#define IO_X2 34
#define IO_X3 39
#define IO_X4 36

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
#define IO_TRIG 23
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
 * @brief 超声波引脚初始化1111
 */
void UltrasonicInit(void);
/**
 * @brief 超声波测距
 *
 * @return int
 */
int UltrasonicDistence(void);
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

long duration;
float distance;

/*
  1 
4   2
  3
*/
uint8_t the_way_arr[128]={1,2,3,4}; //判断方向的数组
int cross_cnt=0; //经过路口的数量，索引

void serialEvent() {
  while (Serial.available()) {
    char incomingChar = Serial.read();
    // 在这里处理接收到的数据
  }
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    delay(500);
    UltrasonicInit();

    /**各种外设初始化**/
    MotorInit();
    LineInit();

    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    Serial.println("oled done");
    display.display();
    delay(2000);

    //    mpu6050.begin();
    //    mpu6050.calcGyroOffsets(true);
    //    Serial.println("mpu6050 done");

    display.clearDisplay();
    /**---------------------TCP Server Init---------------------------**/
    WiFi.mode(WIFI_STA);
    /*------------------connect to the internet-------------------*/
    WiFi.begin(sta_ssid, sta_password);

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(sta_password);
    display.println(sta_ssid);
    display.print("Connecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        display.display();
        delay(500);
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());

    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 30);
    display.println("shl-001");
    //    display.println(WiFi.localIP());
    display.display();

    Udp.begin(3000);
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
}

void loop()
{

    static int err_cnt = 0;
    delay(3);
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
    //SetSpeed(100,0,200);
    tracing();
#if 0
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
        err_cnt = 0;
        digitalWrite(2, HIGH);
        char buf[packetSize + 1];
        String rx;
        Udp.read(buf, packetSize);
        rx = buf;

        index1 = rx.indexOf(':', 0);
        index2 = rx.indexOf(':', index1 + 1);
        index3 = rx.indexOf(':', index2 + 1);
        index4 = rx.indexOf(':', index3 + 1);
        index5 = rx.indexOf(':', index4 + 1);
        index6 = rx.indexOf(':', index5 + 1);
        index7 = rx.indexOf(':', index6 + 1);

        joy_x = rx.substring(0, index1);
        joy_y = rx.substring(index1 + 1, index2);
        but_a = rx.substring(index2 + 1, index3);
        but_b = rx.substring(index3 + 1, index4);
        but_x = rx.substring(index4 + 1, index5);
        but_y = rx.substring(index5 + 1, index6);
        but_l = rx.substring(index6 + 1, index7);
        but_r = rx.substring(index7 + 1, packetSize + 1);

        if (but_y.toInt())
        {
            SetSpeed(100,0,0);
            // SetDirectionAndSpeed(100, 100, 100, 100);
        }
        else if (but_a.toInt())
        {
            SetDirectionAndSpeed(-100, -100, -100, -100);
        }
        else if (but_x.toInt())
        {
            SetDirectionAndSpeed(-100, 100, 100, -100);
        }
        else if (but_b.toInt())
        {
            SetDirectionAndSpeed(100, -100, -100, 100);
        }
        else if (but_l.toInt())
        {
            SetDirectionAndSpeed(-100, 100, -100, 100);
        }
        else if (but_r.toInt())
        {
            SetDirectionAndSpeed(100, -100, 100, -100);
        }
        else
        {
            SetDirectionAndSpeed(0, 0, 0, 0);
        }

        // send_buff[0] = 0xA5;
        // if (joy_x.toInt() == 256)
        //     send_buff[1] = 255;
        // else
        //     send_buff[1] = joy_x.toInt();
        // if (joy_y.toInt() == 256)
        //     send_buff[2] = 255;
        // else
        //     send_buff[2] = joy_y.toInt();
        // send_buff[3] = (but_a.toInt() | (but_b.toInt() << 1) | but_x.toInt() << 2 | but_y.toInt() << 3 | but_l.toInt() << 4 | but_r.toInt() << 5);
        // send_buff[4] = 0x55;
        // Serial.write(send_buff, 5);
    }
    else
    {
        err_cnt++;
        if (err_cnt > 200)
            digitalWrite(2, LOW);
    }
#endif
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
    // int tmp;
    // int max = 150;
    // float factor = 0.0;

    // if (abs(speed1) > max | abs(speed2) > max | abs(speed3) > max | abs(speed4) > max)
    // {
    //     tmp = speed1;
    //     if (abs(speed2) > abs(tmp))
    //         tmp = speed2;
    //     if (abs(speed3) > abs(tmp))
    //         tmp = speed3;
    //     if (abs(speed4) > abs(tmp))
    //         tmp = speed4;
    // }
    // factor = max / tmp;
    // speed1 = speed1 * factor;
    // speed2 = speed2 * factor;
    // speed3 = speed3 * factor;
    // speed4 = speed4 * factor;

    // Serial.println(speed1);
    // Serial.println(speed2);
    // Serial.println(speed3);
    // Serial.println(speed4);

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
}

/**
 * 黑线为0 白线为1
 * x2 x1 x3 x4 是真实分布
 * 0011 数字是 1100 
*/
uint8_t GetLine(void)
{
    uint8_t x1, x2, x3, x4;
    uint8_t tmp = 0;

    x1 = digitalRead(IO_X1);
    x2 = digitalRead(IO_X2);
    x3 = digitalRead(IO_X3);
    x4 = digitalRead(IO_X4); // 黑线为0 白线为1
    Serial.printf("x1: %d, x2: %d, x3: %d, x4: %d\n", x1, x2, x3, x4);

    tmp = x2<<3 | (x1<<2) | (x3 <<1) | x4;
    // tmp = x2 | (x1 << 1) | (x3 << 2) | (x4 << 3);
    return tmp;
}
void tracing(void)
{
    static uint8_t trace_num=0;
    static uint8_t last_trace_num=0;

    last_trace_num=trace_num;
    trace_num = GetLine();

    display_mode(trace_num);
    if(trace_num==15)
    {
        trace_num=last_trace_num; //如果是全白，则重复上次的动作
    }
    switch (trace_num)
    {       
    case 9:  //1001
        SetSpeed(100, 0, 0);
        break;

    case 13:  //1101 右转
        SetSpeed(40, 0, 200);
        break;
    case 14: //1110 右转
        SetSpeed(40, 0, 200);
        break;

    case 11:  //1011 左转
        SetSpeed(40, 0, -200); 
        break;
    case 7:  //0111 左转
        SetSpeed(40, 0, -200); // turn left
        break;
    
    case 0: //0000
    case 1: //0001
    case 8: //1000
    case 3: //0011
    case 12: //1100
    case 2: //0010
    case 4: //0100
        SetSpeed(30, 0, -1000);
        delay(800);
        break;
    default:
        SetSpeed(0, 0, 0);
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
    speed1=speed2=speed3=speed4=0;
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