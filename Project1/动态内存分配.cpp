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
//      printf("���ϸ�:%d\n", *(p + i));
//    }
//  }
//}
//int main()
//{
//  int* p = (int*)malloc(5* sizeof(int));//���ֽڵĸ���
//  //int* p = (int*)calloc(sizeof(arr),sizeof(int));//��������С
//  if (*p == NULL)
//  {
//    printf("err");
//    return -1;
//  }
//
//  ////���ֿռ䲻������
//  //int* ptr = (int*)realloc(p, 50);//���ڵ��ڴ��ַ�����ڳɶ����ֽ�
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
//  //�ͷ��ڴ�
//  free(p);
//  p = NULL;
//  return 0;
//}