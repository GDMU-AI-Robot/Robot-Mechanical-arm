#include <Servo.h>

//  初始化机械臂参数  单位为度数
#define    Move_A      20
#define    Move_B      160

#define    Move_C      150
#define    Move_D      0
#define    Move_E      55

//      到达地面参数    单位为度数
#define    Move_A_Down      40
#define    Move_B_Down      160

#define    Move_C_Down      5
#define    Move_D_Down      50  //38
#define    Move_E_Down      55

Servo servo1;  // 创建Servo对象来控制第一个伺服舵机
Servo servo2;  // 创建Servo对象来控制第二个伺服舵机

Servo POINT_A;
Servo POINT_B;
Servo POINT_C;
Servo POINT_D;
Servo POINT_E;

int angle = 0;

void setup() {
  Serial.begin(115200); // 使用默认的串口引脚进行通信

  //机械臂初始化
  POINT_A.attach(3, 500, 2500);
  POINT_B.attach(5, 500, 2500);
  POINT_C.attach(9, 500, 2500);
  POINT_D.attach(10, 500, 2500);
  POINT_E.attach(11, 500, 2500);

  // POINT_A.attach(3);
  // POINT_B.attach(5);
  // POINT_C.attach(9);
  // POINT_D.attach(10);
  // POINT_E.attach(11);

  servo1.attach(6); // 左轮 6
  servo2.attach(7); // 右轮 7

  
  //Serial.println("1111111111111111");
  // 设置初始停止状态
  stop();
  Init_Hand();

}

void loop() {
  // Serial.println("hello");
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.println(cmd);
    switch (cmd) {
      case 'F':
        moveForward();
        // Serial.println("111");
        break;
      case 'B':
        moveBack();
        break;
      case 'L':
        moveLeft();
        break;
      case 'R':
        moveRight();
        break;

      //放下机械臂
      case 'D':
        Down();
        break;
      //抓取物块
      case 'G':
        Hand_Getting();
        break;

      //松开
      case 'O':
        Hand_NoGetting();
        break;

      //抬起来  
      case 'U':
        Up();
        break;

      case 'S':
        stop();     
        break; // 
    }
  }
}

void StandardSpeed(int a, int b) {
  servo1.writeMicroseconds(1580 + a * 10); // 1865 数值增大为前进，数值减小为后退
  servo2.writeMicroseconds(1460 - b * 10); // 785 数值减小为前进，数值增大为后退
}

void moveForward() {
  StandardSpeed(25, 25);
}

void moveBack() {
  StandardSpeed(-25, -25);
}

void moveLeft() {
  StandardSpeed(-25, 25);
}

void moveRight() {
  StandardSpeed(25, -25);
}

void stop() {
  // 调整为停止的脉冲宽度
  servo1.writeMicroseconds(1585); // 设置servo1到停止位置   1580往大的调就是往前，反之
  servo2.writeMicroseconds(1460);  // 设置servo2到停止位置  1460

  // servo1.writeMicroseconds(1700); // 设置servo1到停止位置   1580往大的调就是往前，反之
  // servo2.writeMicroseconds(1300);  // 设置servo2到停止位置  1460
}

void Init_Hand()
{
  POINT_A.write(Move_A);
  POINT_B.write(Move_B);
  POINT_C.write(Move_C);
  POINT_D.write(Move_D);
  POINT_E.write(Move_E);
}

void Down()
{
  POINT_A.write(Move_A);
  POINT_B.write(Move_B);
  InputAngle(Move_C,Move_C_Down,POINT_C);
  delay(500); 
  InputAngle(Move_D,Move_D_Down,POINT_D);
  // POINT_D.write(Move_D);
}

void Up()
{
  POINT_A.write(Move_A);
  POINT_B.write(Move_B);
  InputAngle(Move_D_Down,Move_D,POINT_D);
  delay(500); 
  InputAngle(Move_C_Down,Move_C,POINT_C);
  delay(1000); 
}

void Hand_Getting()
{
  POINT_E.write(118);
}

void Hand_NoGetting()
{
  POINT_E.write(55);
}


void InputAngle(int set,int tar,class Servo ONE)
{
  //增大角度
  if(set < tar)
  {
    for (angle = set; angle <= tar; angle++) 
    {  
    ONE.write(angle);  
    delay(15);  
    }
  }

  //减小角度
  if(set > tar)
  {
    for (angle = set; angle >= tar; angle--) 
    { 
    ONE.write(angle);
    delay(15);
    }
  }
}
