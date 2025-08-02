

#define ENA    5                                                                                                                                                        // Enable/speed motors Right          GPIO 5(D1)
//#define ENA_2  4          // Enable/speed motors Right_2        GPIO 4(D2)
#define ENB    0          // Enable/speed motors Left           GPIO 0(D3)
//#define ENB_2  2          // Enable/speed motors Left_2         GPIO 2(D4)
#define IN_1  14          // L298N in1 motors Right             GPIO14(D5)
#define IN_2  12          // L298N in2 motors Right             GPIO12(D6)
#define IN_3  13          // L298N in3 motors Left              GPIO13(D7)
#define IN_4  15          // L298N in4 motors Left              GPIO15(D8)
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
 
/* Wifi ssid and pwd */
const char *ssid = "MY8266CAR";
const char *password = "12345678";
 
/* your car speed */
int speedCar = 0;         // 0 - 1023.  450 
int speedInterval = 50;     //速度间隔
int speedTrun = 1.5;        //转弯系数  系数越大 左右两边转弯差速越大
 
ESP8266WebServer server(80);
String webPage = "";
 
/* 192.168.4.1 */
void handleRoot() {
  server.send(200, "text/html", "<h1>You are connected</h1>");
}
 
void setup() {
  webPage += "<div align=\"center\"><h1>ESP8266 CAR</h1>";
  webPage += "<a href=\"FL\"><button style=\"height:200px;width:200px\"><font size=\"20\">FL</font></button></a>";
  webPage += "<a href=\"F2\"><button style=\"height:200px;width:200px\"><font size=\"20\">FW</font></button></a>";
  webPage += "<a href=\"FR\"><button style=\"height:200px;width:200px\"><font size=\"20\">FR</font></button></a><br>";
  webPage += "<a href=\"L2\"><button style=\"height:200px;width:200px\"><font size=\"20\">LT</font></button></a>";
  webPage += "<a href=\"S2\"><button style=\"height:200px;width:200px\"><font size=\"20\">ST</font></button></a>";
  webPage += "<a href=\"R2\"><button style=\"height:200px;width:200px\"><font size=\"20\">RT</font></button></a><br>";
  webPage += "<a href=\"BL\"><button style=\"height:200px;width:200px\"><font size=\"20\">BL</font></button></a>";
  webPage += "<a href=\"B2\"><button style=\"height:200px;width:200px\"><font size=\"20\">BC</font></button></a>";
  webPage += "<a href=\"BR\"><button style=\"height:200px;width:200px\"><font size=\"20\">BR</font></button></a></div>";
  webPage += "<div align=\"center\"><h1>SPEED CHANGE</h1>";
  webPage += "<a href=\"SE1\"><button style=\"height:200px;width:200px\"><font size=\"20\">1</font></button></a>";
  webPage += "<a href=\"SE2\"><button style=\"height:200px;width:200px\"><font size=\"20\">2</font></button></a>";
  webPage += "<a href=\"SE3\"><button style=\"height:200px;width:200px\"><font size=\"20\">3</font></button></a><br>";
  webPage += "<a href=\"SE4\"><button style=\"height:200px;width:200px\"><font size=\"20\">4</font></button></a>";
  webPage += "<a href=\"SE5\"><button style=\"height:200px;width:200px\"><font size=\"20\">5</font></button></a>";
  webPage += "<a href=\"SE6\"><button style=\"height:200px;width:200px\"><font size=\"20\">6</font></button></a><br>";
  webPage += "<a href=\"SE7\"><button style=\"height:200px;width:200px\"><font size=\"20\">7</font></button></a>";
  webPage += "<a href=\"SE8\"><button style=\"height:200px;width:200px\"><font size=\"20\">8</font></button></a>";
  webPage += "<a href=\"SE9\"><button style=\"height:200px;width:200px\"><font size=\"20\">9</font></button></a>";
 
  // preparing GPIOs
  pinMode(ENA, OUTPUT);
  //digitalWrite(ENA, 0);
  pinMode(ENB, OUTPUT);
  //digitalWrite(ENB, 0);
  pinMode(IN_1, OUTPUT);
  // digitalWrite(IN_1, 0);
  pinMode(IN_2, OUTPUT);
  // digitalWrite(IN_2, 0);
  pinMode(IN_3, OUTPUT);
  // digitalWrite(IN_3, 0);
  pinMode(IN_4, OUTPUT);
  // digitalWrite(IN_4, 0);
 
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
 
  server.on("/", []() {
    server.send(200, "text/html", webPage);
  });
  server.on("/FL", []() {
    server.send(200, "text/html", webPage);
    FL();
  });
  server.on("/F2", []() {
    server.send(200, "text/html", webPage);
    F2();
  });
  server.on("/FR", []() {
    server.send(200, "text/html", webPage);
    FR();
  });
  server.on("/L2", []() {
    server.send(200, "text/html", webPage);
    L2();
  });
  server.on("/S2", []() {
    server.send(200, "text/html", webPage);
    S2();
  });
  server.on("/R2", []() {
    server.send(200, "text/html", webPage);
    R2();
  });
  server.on("/BL", []() {
    server.send(200, "text/html", webPage);
    BL();
  });
  server.on("/B2", []() {
    server.send(200, "text/html", webPage);
    B2();
  });
  server.on("/BR", []() {
    server.send(200, "text/html", webPage);
    BR();
  });
 
  server.on("/SE1", []() {
    server.send(200, "text/html", webPage);
    CS(1);
  });
  server.on("/SE2", []() {
    server.send(200, "text/html", webPage);
    CS(2);
  });
  server.on("/SE3", []() {
    server.send(200, "text/html", webPage);
    CS(3);
  });
  server.on("/SE4", []() {
    server.send(200, "text/html", webPage);
    CS(4);
  });
  server.on("/SE5", []() {
    server.send(200, "text/html", webPage);
    CS(5);
  });
  server.on("/SE6", []() {
    server.send(200, "text/html", webPage);
    CS(6);
  });
  server.on("/SE7", []() {
    server.send(200, "text/html", webPage);
    CS(7);
  });
  server.on("/SE8", []() {
    server.send(200, "text/html", webPage);
    CS(8);
  });
  server.on("/SE9", []() {
    server.send(200, "text/html", webPage);
    CS (9);
  });
 
  server.begin();
  Serial.println("HTTP server started");
}
 
/*******************************************/

//左转90度
void FL() {
  analogWrite(ENA, 300);
  analogWrite(ENB, 300);
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 1);
 
  digitalWrite(IN_3, 1);
  digitalWrite(IN_4, 0);

  delay(705);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
 
}
 
void F2() {
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar);
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 1);
 
  digitalWrite(IN_3, 0);
  digitalWrite(IN_4, 1);
 
}
 
 //右转90度
void FR() {
  analogWrite(ENA, 300);
  analogWrite(ENB, 300);
  digitalWrite(IN_1, 1);
  digitalWrite(IN_2, 0);
 
  digitalWrite(IN_3, 0);
  digitalWrite(IN_4, 1);

  delay(705);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}
 
void L2() {
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar);
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 1);
 
  digitalWrite(IN_3, 1);
  digitalWrite(IN_4, 0);
}
 
void S2() {
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 0);
  digitalWrite(IN_3, 0);
  digitalWrite(IN_4, 0);
}
 
void R2() {
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar);
  digitalWrite(IN_1, 1);
  digitalWrite(IN_2, 0);
 
  digitalWrite(IN_3, 0);
  digitalWrite(IN_4, 1);
}
 
void BL() {
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar / speedTrun);
  digitalWrite(IN_1, 1);
  digitalWrite(IN_2, 0);
 
  digitalWrite(IN_3, 1);
  digitalWrite(IN_4, 0);
}
 
void B2() {
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar);
  digitalWrite(IN_1, 1);
  digitalWrite(IN_2, 0);
 
  digitalWrite(IN_3, 1);
  digitalWrite(IN_4, 0);
}
 
void BR() {
  analogWrite(ENA, speedCar / speedTrun);
  analogWrite(ENB, speedCar);
  digitalWrite(IN_1, 1);
  digitalWrite(IN_2, 0);
 
  digitalWrite(IN_3, 1);
  digitalWrite(IN_4, 0);
}
 
void CS(int NUM) {
  //speedCar = 1023 - (9 - NUM) * speedInterval;
  speedCar =50 + NUM * speedInterval;
  analogWrite(ENA, speedCar);
  analogWrite(ENB, speedCar);
}
 
/*******************************************/
 
void loop() {
  server.handleClient();
}
