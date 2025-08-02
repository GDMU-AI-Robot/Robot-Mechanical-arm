// #include <WiFi.h>
// #include <WebServer.h>

// const char* ssid = "toutou_DYDY";
// const char* password = "SHUANGDANHUANGDYDY.";

// WebServer server(80);

// void setup() {
//   Serial.begin(115200);

//   // WiFi.softAP(ssid, password);
//   // 设置WiFi AP并指定IP地址

//   IPAddress ip(192, 168, 4, 4);
//   WiFi.softAP(ssid, password, 1, 0, 4, 0);  // 这里可以设置信道、隐藏等参数
//   WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0)); // 配置IP地址


//   server.on("/", HTTP_GET, []() {
//     server.send(200, "text/html", R"(
//       <html><head><title>Car Control</title></head><body>
//       <h1>Halo,this is Orange!</h1>
//       <script>
//         document.body.onkeydown = function(e) {
//           if (e.keyCode == 87) { fetch('/forward'); }
//           else if (e.keyCode == 83) { fetch('/backward'); }
//           else if (e.keyCode == 65) { fetch('/left'); }
//           else if (e.keyCode == 68) { fetch('/right'); }
//           else if (e.keyCode == 74) { fetch('/sendD'); } // J
//           else if (e.keyCode == 75) { fetch('/sendG'); } // K
//           else if (e.keyCode == 76) { fetch('/sendU'); } // L
//           else if (e.keyCode == 73) { fetch('/sendO'); } // I
//         }
//       </script></body></html>
//     )");
//   });

//   server.on("/forward", HTTP_GET, []() {
//     Serial.write('F');
//     server.send(200, "text/plain", "Command to move forward sent");
//   });

//   server.on("/backward", HTTP_GET, []() {
//     Serial.write('B');
//     server.send(200, "text/plain", "Command to move backward sent");
//   });

//   server.on("/left", HTTP_GET, []() {
//     Serial.write('L');
//     server.send(200, "text/plain", "Command to move left sent");
//   });

//   server.on("/right", HTTP_GET, []() {
//     Serial.write('R');
//     server.send(200, "text/plain", "Command to move right sent");
//   });

//   // 添加 J, K, L 键的路由
//   server.on("/sendD", HTTP_GET, []() {
//     Serial.write('D');
//     server.send(200, "text/plain", "Command D sent");
//   });

//   server.on("/sendG", HTTP_GET, []() {
//     Serial.write('G');
//     server.send(200, "text/plain", "Command G sent");
//   });

//   server.on("/sendU", HTTP_GET, []() {
//     Serial.write('U');
//     server.send(200, "text/plain", "Command U sent");
//   });

//   // 添加 I 键的路由
//   server.on("/sendO", HTTP_GET, []() {
//     Serial.write('O');
//     server.send(200, "text/plain", "Command O sent");
//   });

//   server.begin();
// }

// void loop() {
//   server.handleClient();
// }


#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "LAPTOP-MIO393FS 6436"; // 修改为你电脑的热点SSID
const char* password = "12345678";  // 修改为你电脑的热点密码

WebServer server(80);

void setup() {
  Serial.begin(115200);
  
  // 连接到 WiFi
  WiFi.begin(ssid, password);

  // 等待连接
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi!");

  // 设置服务器路由
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", R"(
      <html><head><title>Car Control</title></head><body>
      <h1>Halo, this is Orange!</h1>
      <script>
        document.body.onkeydown = function(e) {
          if (e.keyCode == 87) { fetch('/forward'); }
          else if (e.keyCode == 83) { fetch('/backward'); }
          else if (e.keyCode == 65) { fetch('/left'); }
          else if (e.keyCode == 68) { fetch('/right'); }
          else if (e.keyCode == 74) { fetch('/sendD'); } // J
          else if (e.keyCode == 75) { fetch('/sendG'); } // K
          else if (e.keyCode == 76) { fetch('/sendU'); } // L
          else if (e.keyCode == 73) { fetch('/sendO'); } // I
        }
      </script></body></html>
    )");
  });

  server.on("/forward", HTTP_GET, []() {
    Serial.write('F');
    server.send(200, "text/plain", "Command to move forward sent");
  });

  server.on("/backward", HTTP_GET, []() {
    Serial.write('B');
    server.send(200, "text/plain", "Command to move backward sent");
  });

  server.on("/left", HTTP_GET, []() {
    Serial.write('L');
    server.send(200, "text/plain", "Command to move left sent");
  });

  server.on("/right", HTTP_GET, []() {
    Serial.write('R');
    server.send(200, "text/plain", "Command to move right sent");
  });

  // 添加 J, K, L 键的路由
  server.on("/sendD", HTTP_GET, []() {
    Serial.write('D');
    server.send(200, "text/plain", "Command D sent");
  });

  server.on("/sendG", HTTP_GET, []() {
    Serial.write('G');
    server.send(200, "text/plain", "Command G sent");
  });

  server.on("/sendU", HTTP_GET, []() {
    Serial.write('U');
    server.send(200, "text/plain", "Command U sent");
  });

  // 添加 I 键的路由
  server.on("/sendO", HTTP_GET, []() {
    Serial.write('O');
    server.send(200, "text/plain", "Command O sent");
  });
  
  server.begin();
}

void loop() {
  server.handleClient();
}
