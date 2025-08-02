//#include <stdio.h>
//
////p291第四题
////1 2 3 4 5 6 7 8 9 10
//int main()
//{
//  void input(int* p,int n);
//  int* handle(int* p,int n,int m);
//  void print(int* p,int n);
//
//  int arr[10] = { 0 };
//  int* p = arr;
//  int n, m;
//
//  scanf_s("%d %d", &n, &m);
//  input(p,n);
//  print(p,n);
//  int *yes = handle(p,n,m);
//  print(yes,n);
//  return 0;
//
//}
//
//int *handle(int* p,int n,int m)
//{
//  int R[20] = { 0 };
//  int * result=R;
//  int i;
//  for (i = 0; i < n - m; i++)
//  {
//    *(R + i + m) = *(p + i);
//  }
//  int j;
//  for (j=0,i = n - m; i < n; i++,j++)
//  {
//    *(R + j) = *(p + i);
//  }
//
//  return result;
//}
//
//
//void input(int *p,int n)
//{
//  int i;
//  for (i = 0;i< n;i++)
//  {
//    scanf_s("%d", (p + i));
//  }
//}
//
//void print(int* p,int n)
//{
//  int i;
//  for (i = 0; i < n; i++)
//  {
//    printf("%d\t", *(p + i));
//  }
//  printf("\n");
//}
//
//////P291 第三题 对换数据
////void input(int* p);
////void print(int* p);
////void handle(int* p);
////int main()
////{
////  int arr[10] = { 0 };
////  input(arr);
////  print(arr);
////  handle(arr);
////  print(arr);
////  return 0;
////}
////
////void input(int* p)
////{
////  int i;
////  printf("请输入10个整数：");
////  for (i = 0; i < 10; i++)
////  {
////    scanf_s("%d", p + i);
////  }
////}
////
////void print(int* p)
////{
////  int i;
////  for (i = 0; i < 10; i++)
////  {
////    printf("%d\t", *(p + i));
////  }
////  printf("\n");
////}
////
////void handle(int* p)
////{
////  int i;
////  int min_idx = 0;  // 最小值索引
////  int max_idx = 0;  // 最大值索引
////
////  // 找到最小值和最大值的索引
////  for (i = 0; i < 10; i++)
////  {
////    if (*(p + i) < *(p + min_idx))
////    {
////      min_idx = i;
////    }
////    if (*(p + i) > *(p + max_idx))
////    {
////      max_idx = i;
////    }
////  }
////
////  // 交换最小值到数组首位
////  int tmp = *(p + min_idx);
////  *(p + min_idx) = *p;
////  *p = tmp;
////
////  // 关键修正：如果最大值原本在首位，交换后它的位置变为min_idx
////  if (max_idx == 0)
////  {
////    max_idx = min_idx;
////  }
////
////  // 交换最大值到数组末尾
////  tmp = *(p + max_idx);
////  *(p + max_idx) = *(p + 9);
////  *(p + 9) = tmp;
////}
//
//
////int main()
////{
////  const char* p = "abcd";  // p指向字符串常量"abcd"
////  p = "aaere ";      // p重新指向字符串常量"aaere "（末尾有空格）
////  printf("%s", p);   // 输出p当前指向的字符串
////
////  return 0;
////}
//
////int main()
////{
////  int arr[3][4] = { 0,1,2,3,
////    4,5,6,7,
////    8,9,10,11 };
////
////  int(*p)[4];  // 正确的指针声明语法，*与p必须紧密结合
////  p = arr;      // 现在p的类型与arr匹配，可以正确赋值
////  printf("%d,%d,%d\n", **arr, *(*(p + 1)+1), **((arr + 1) + 1));
////
////
////  return 0;
////}