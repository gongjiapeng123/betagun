#include <FlexiTimer2.h>
//#include "DHT11_YYROBOT.h"

#define debug 0
#define debug_infrared 0
#define debug_hall 0

class HCSR04
{
public:
	HCSR04(byte trigPin, byte echoPin)
	{
		_trigPin = trigPin;
		_echoPin = echoPin;
		pinMode(echoPin, INPUT);
		Distance = 0;
	}
	u16 GetDistance()
	{
		digitalWrite(_trigPin, LOW);
		delayMicroseconds(2);
		// 10us高电平触发
		digitalWrite(_trigPin, HIGH);
		delayMicroseconds(10);
		digitalWrite(_trigPin, LOW);
		// 返回微秒
		u16 distance = pulseIn(_echoPin, HIGH);
		Distance = distance / 58;  // cm

		return Distance;  //cm
	}

	u16 Distance;

private:
	byte _trigPin;
	byte _echoPin;
};

const int CODED_DISC_GRID_NUM = 50;  // 码盘栅格数
const float WHEEL_DIAMETER = 0.04;  // 轮子直径(米)
const float RESOLUTION = 0.001;  // FlexiTimer2分辨率(0.001s)
const int VHZ = 20;  // 1秒中断测速几次
const int FLEXITIMER2UNIT = (int)(1 / RESOLUTION / VHZ);  // FlexiTimer2 unit

//DHT11 myDHT11(8);		//DHT11接在D8引脚

HCSR04 HCSR04_0(23, 22);  // 超声波传感器

byte d0_down_pin = 53, d1_down_pin = 51, d2_down_pin = 49, d3_down_pin = 47;  // 向下红外
byte d4_front_pin = 52, d5_front_pin = 50, d6_front_pin = 48, d7_front_pin = 48;  // 向前红外
/**
 * 红外避障传感器的输入值，D0, D1, D2, D3为向下的传感器，
 * D4, D5, D6, D7为向前，排列顺序为以左车头开始顺时针，
 * 数值为0说明距离小于阈值（led灯亮），为1说明距离大于阈值。
 * D0, D1, D2, D3期望为0，指示没有远离地面（以防移动后坠落），
 * D4, D5, D6, D7期望为1，指示前方没有障碍物（以防移动后碰撞）。
 */
bool D0, D1, D2, D3, D4, D5, D6, D7; 

byte hall_sensor_d1_interrupt_num = 0;  // 左电机测速中断号，高电平灭
byte hall_sensor_d2_interrupt_num = 1;  // 右电机测速中断号
int left_count, right_count;  // 码盘计数
float left_velocity, right_velocity;  // 测速结果

// 计算数据包校验和
byte calCheckSum(byte data[11])
{
	char res = 0;
	for (size_t i = 0; i < 10; i++)
	{
		res += data[i];
	}
	return res;
}

void setup()
{
	Serial.begin(115200);  // 设置通讯的波特率

	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}

	Serial.println("Serial initialized");

	pinMode(d0_down_pin, INPUT);
	pinMode(d1_down_pin, INPUT);
	pinMode(d2_down_pin, INPUT);
	pinMode(d3_down_pin, INPUT);
	pinMode(d4_front_pin, INPUT);
	pinMode(d5_front_pin, INPUT);
	pinMode(d6_front_pin, INPUT);
	pinMode(d7_front_pin, INPUT);

    // 中断计数
    attachInterrupt(hall_sensor_d1_interrupt_num, leftCount, FALLING);
    attachInterrupt(hall_sensor_d2_interrupt_num, rightCount, FALLING);

    // 100ms进行一次计算速度
    FlexiTimer2::set(FLEXITIMER2UNIT, RESOLUTION, countHandler);
    FlexiTimer2::start();
}

void countHandler() {

#if debug
#if debug_hall
    Serial.print(left_count);
    Serial.print(" ");
    Serial.print(right_count);
    Serial.println();
#endif
#else
    // 发放计数信息给上位机进行计算
    sendPacket(0x53);
#endif
    // 清空
    left_count = 0;
    right_count = 0;    
}

float getVelocity(int count) {
    float vel = (count / CODED_DISC_GRID_NUM) * (WHEEL_DIAMETER * PI) * VHZ;
    return vel;  // m/s
}

float leftCount() {
    left_count++;
}

void rightCount() {
    right_count++;
}

void loop()
{
	//delayMicroseconds(1000);
	// 读取温湿度传感器值，经过这个函数后，myDHT11.TEM_Buffer_Int和myDHT11.HUMI_Buffer_Int被分别填充上了温度和湿度值
	// myDHT11.DHT11_Read();

	// 获取红外避障传感器数据，0表示距离小于阈值，1表示大于阈值
	D0 = digitalRead(d0_down_pin);
	D1 = digitalRead(d1_down_pin);
	D2 = digitalRead(d2_down_pin);
	D3 = digitalRead(d3_down_pin);
	D4 = digitalRead(d4_front_pin);
	D5 = digitalRead(d5_front_pin);
	D6 = digitalRead(d6_front_pin);
	D7 = digitalRead(d7_front_pin);

	// 获取超声波传感器数据
	// HCSR04_0.GetDistance();
	// HCSR04_1.GetDistance();
	
    // 获取测速计数器
    
#if debug
	// Serial.print("HUMI = ");
	// Serial.print(myDHT11.HUMI_Buffer_Int);
	// Serial.println(" %RH");
	// Serial.print("TMEP = ");
	// Serial.print(myDHT11.TEM_Buffer_Int);
	// Serial.println(" C");

	// Serial.print("DIST0 = ");
	// Serial.print(HCSR04_0.Distance);
	// Serial.println(" CM");
	// Serial.print("DIST1 = ");
	// Serial.print(HCSR04_1.Distance);
	// Serial.println(" CM");

#if debug_infrared
	D0 ? Serial.print("1") : Serial.print("0");
	D1 ? Serial.print("1") : Serial.print("0");
	D2 ? Serial.print("1") : Serial.print("0");
	D3 ? Serial.print("1") : Serial.print("0");
	Serial.println();
	D4 ? Serial.print("1") : Serial.print("0");
	D5 ? Serial.print("1") : Serial.print("0");
	D6 ? Serial.print("1") : Serial.print("0");
	D7 ? Serial.print("1") : Serial.print("0");
	Serial.println();
#endif
	
#else
	
    sendPacket(0x50);
	sendPacket(0x51);
	sendPacket(0x52);
#endif // debug

}

/**
 * 数据头  数据包类型           数据                    校验和
 * 0x55     0x5*             8  byte                  1  byte
 *
 * 1、DHT11（温湿度）：0x55     0x50      HUMI TEMP 0x0 0x0 0x0 0x0 0x0 0x0       校验和
 * HUMI是湿度，TEMP是温度，0x0 表示留空
 *
 * 2、超声波：0x55     0x51      DL0 DH0 DL1 DH1 DL2 DH2 DL3 DH3        校验和
 * DLX DHX 表示第X个超声波传感器获得的距离值，以小端short方式存储
 *
 * 3、红外：0x55 0x52 S 0x0 0x0 0x0 0x0 0x0 0x0 0x0 SUM
 * S: 第X位表示第X个红外传感器的值，0x0 表示留空，即：
 * 
 *
 * 4、霍尔测速：0x55 0x53 left_count right_count 0x0 0x0 0x0 0x0 0x0 0x0       校验和
 */
void sendPacket(byte packId) {
    byte packet[11] = { 0x55 };
    switch (packId) {
        case 0x50: {  // 发送50数据包，温湿度
            packet[1] = packId;
            packet[2] = 0;
            packet[3] = 0;
            packet[4] = packet[5] = packet[6] = packet[7] = packet[8] = packet[9] = 0x00;
            packet[10] = calCheckSum(packet);
            Serial.write(packet, 11);
        }
            break;
        case 0x51: {  // 红外
            packet[1] = packId;
            packet[2] = (D0 << 7) + (D1 << 6) + (D2 << 5) + (D3 << 4) + (D4 << 3) + (D5 << 2) + (D6 << 1) + D7;
            packet[3] = packet[4] = packet[5] = packet[6] = packet[7] = packet[8] = packet[9] = 0x00;
	        packet[10] = calCheckSum(packet);
	        Serial.write(packet, 11);
        }
            break;
        case 0x52: {  // 超声
            packet[1] = packId;
            packet[2] = HCSR04_0.Distance & 0x00FF;  // arduino是大端的，此处将数据转为小端传输
            packet[3] = HCSR04_0.Distance >> 8;
            packet[4] = 0x00;
            packet[5] = 0x00;
            packet[6] = packet[7] = packet[8] = packet[9] = 0x00;
            packet[10] = calCheckSum(packet);
            Serial.write(packet, 11);
        }
            break;
        case 0x53: {  // 测速
            packet[1] = packId;
            packet[2] = left_count & 0x00FF;
            packet[3] = left_count >> 8;
            packet[4] = right_count & 0x00FF;
            packet[5] = right_count >> 8;
            packet[6] = packet[7] = packet[8] = packet[9] = 0x00;
            packet[10] = calCheckSum(packet);
            Serial.write(packet, 11);
        }
            break;
    }
}

