//#include <stdio.h>
//
////p291������
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
//////P291 ������ �Ի�����
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
////  printf("������10��������");
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
////  int min_idx = 0;  // ��Сֵ����
////  int max_idx = 0;  // ���ֵ����
////
////  // �ҵ���Сֵ�����ֵ������
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
////  // ������Сֵ��������λ
////  int tmp = *(p + min_idx);
////  *(p + min_idx) = *p;
////  *p = tmp;
////
////  // �ؼ�������������ֵԭ������λ������������λ�ñ�Ϊmin_idx
////  if (max_idx == 0)
////  {
////    max_idx = min_idx;
////  }
////
////  // �������ֵ������ĩβ
////  tmp = *(p + max_idx);
////  *(p + max_idx) = *(p + 9);
////  *(p + 9) = tmp;
////}
//
//
////int main()
////{
////  const char* p = "abcd";  // pָ���ַ�������"abcd"
////  p = "aaere ";      // p����ָ���ַ�������"aaere "��ĩβ�пո�
////  printf("%s", p);   // ���p��ǰָ����ַ���
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
////  int(*p)[4];  // ��ȷ��ָ�������﷨��*��p������ܽ��
////  p = arr;      // ����p��������arrƥ�䣬������ȷ��ֵ
////  printf("%d,%d,%d\n", **arr, *(*(p + 1)+1), **((arr + 1) + 1));
////
////
////  return 0;
////}