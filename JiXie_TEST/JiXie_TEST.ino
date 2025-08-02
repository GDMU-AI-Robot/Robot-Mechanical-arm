#include <Servo.h>
//初始化
//1 - 5度
//2 - 150度
//3 - 0度
//4 - 55度 （开）

//放在地上
//1 - 5度
//2 - 20度
//3  - 38度（水平直的）
//4   - 55度（开）

//放在地上加=夹取
//1 - 5度
//2 - 20度
//3  - 38度（水平直的）
//4   - 125度（夹）

//  初始化机械臂参数  单位为度数
#define    Move_A      20
#define    Move_B      160

#define    Move_C      150
#define    Move_D      0
#define    Move_E      55

//      到达地面参数    单位为度数
#define    Move_A_Down      40
#define    Move_B_Down      160

#define    Move_C_Down      20
#define    Move_D_Down      38
#define    Move_E_Down      55

Servo servoLeft;  // 创建Servo对象来控制第一个伺服舵机
Servo servoRight;  // 创建Servo对象来控制第二个伺服舵机

Servo POINT_A;
Servo POINT_B;
Servo POINT_C;
Servo POINT_D;
Servo POINT_E ;

int angle = 0; 

void setup() {

  // POINT_A.attach(2);
  // POINT_B.attach(3);
  // POINT_C.attach(4);
  // POINT_D.attach(5);
  // POINT_E.attach(6);




  POINT_A.attach(3, 500, 2500);
  POINT_B.attach(5, 500, 2500);
  POINT_C.attach(9, 500, 2500);
  POINT_D.attach(10, 500, 2500);
  POINT_E.attach(11, 500, 2500);

  Init_Hand();
}

void loop() 
{       
    // for (angle = 100; angle <= 180; angle++) //100->180
    // {  
    // POINT_A.write(angle);  
    // delay(15);  
    // }

    // for (angle = 180; angle >= 100; angle--) //180->0
    // { 
    // POINT_A.write(angle);
    // delay(15);
    // }

    // POINT_A.write(55);
    // delay(1000); 
    // POINT_A.write(125);
    // delay(2000); 

    //一整套动作
    Init_Hand();
    delay(3000);
    Down();
    delay(3000);
    Hand_Getting();
    delay(3000);
    Up();
    delay(3000);

    while(1);
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


