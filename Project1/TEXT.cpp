#include <stdio.h>

int main()
{
  int a = 2;
  char str[20] ="Hello world!\n";

  //表达式区域
  //a *=2 + 3;//a=2
  a = 0;
  printf("%d\n", a=3!=2);
  printf("%d\n", (a =3) != 2);
  //打印区域
  printf("整型是：%d\n浮点型是：%f\n", a, a);
  printf(str);
  return 0;
}

