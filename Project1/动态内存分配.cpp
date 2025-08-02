//#include <stdio.h>
//#include <stdlib.h>
//
//void check(int* p, int sz)
//{
//  int i;
//  for(i = 0; i < sz; i++)
//  {
//    if (*(p + i) < 60)
//    {
//      printf("不合格:%d\n", *(p + i));
//    }
//  }
//}
//int main()
//{
//  int* p = (int*)malloc(5* sizeof(int));//总字节的个数
//  //int* p = (int*)calloc(sizeof(arr),sizeof(int));//个数，大小
//  if (*p == NULL)
//  {
//    printf("err");
//    return -1;
//  }
//
//  ////发现空间不够用了
//  //int* ptr = (int*)realloc(p, 50);//调节的内存地址，调节成多少字节
//  //if (ptr != NULL)
//  //{
//  //  p = ptr;
//  //}
//
//  int i;
//  for (i = 0; i < 5; i++)
//  {
//    scanf_s("%d", (p + i));
//  }
//  check(p, 5);
//
//  //释放内存
//  free(p);
//  p = NULL;
//  return 0;
//}