//#include <stdio.h>
//#include <stdlib.h>
//
//#define LEN sizeof(struct Student)
//
//struct Student
//{
//  int num;
//  struct Student* next;
//};
//int n;
//
//struct Student* creat(void)
//{
//  struct Student* head, * p1, * p2;
//  n = 0;
//  p1 = p2 = (struct Student*)malloc(LEN);
//  scanf_s("%d", &p1->num);
//  head = NULL;
//
//  while (p1->num != 0)
//  {
//    n = n + 1;
//    if (n == 1)head = p1;
//    else p2->next = p1;
//    p2 = p1;
//
//    p1 = (struct Student*)malloc(LEN);
//    scanf_s("%d", &p1->num);
//  }
//  p2->next = NULL;
//  return head;
//}
//
//int main() 
//{
//  struct Student* pa;
//  pa = creat();
//  if (pa != NULL)
//    do
//    {
//      printf("%d->", pa->num);
//      pa = pa->next;
//    } while (pa != NULL);
//}

////静态
//#include <stdio.h>
//
//int main()
//{
//  struct stu
//  {
//    char name[20];
//    int num;
//    char sex[10];
//    struct stu* next;
//  };
//  struct stu s1 = { "小明", 12, "男" };
//  struct stu s2 = { "小海", 20, "男" };
//  struct stu s3 = { "小莹", 21, "女" };
//
//  struct stu* head, * p;
//  head = &s1;
//  s1.next = &s2;
//  s2.next = &s3;
//  s3.next = NULL;
//
//  p = head;
//  while (p != NULL)
//  {
//    printf("%s %d %s\n", p->name, p->num, p->sex);
//    p = p->next;
//  }
//  // 打印结构体变量的地址
//  printf("结构体s1的地址: %p\n", &s1);
//  printf("name成员地址: %p\n", &s1.name);
//  printf("num成员地址: %p\n", &s1.num);
//  printf("sex成员地址: %p\n", &s1.sex);
//
//  return 0;
//}