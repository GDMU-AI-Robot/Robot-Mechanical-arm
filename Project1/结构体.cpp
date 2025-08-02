//#define _CRT_SECURE_NO_WARNINGS
//#include <stdio.h>
//
//int main()
//{
//  struct stu
//  {
//    char name[20];
//    int num;
//    float score[2];  // 存储两门成绩
//  };
//  struct stu* p;
//  struct stu arr[2];  // 可以存储2个学生信息
//  p = arr;  // 让指针p指向数组arr的首地址
//
//  for (int i = 0; i < 2; i++)
//  {
//    printf("请输入第%d个学生的信息(姓名 学号 成绩1 成绩2): ", i + 1);
//    // 使用scanf_s时，对于字符串需要指定缓冲区大小
//    //scanf_s("%s %d %f %f", arr[i].name, sizeof(arr[i].name),
//    //  &arr[i].num, &arr[i].score[0], &arr[i].score[1]);
//    scanf("%s %d %f %f", (p + i)->name,
//      &(p + i)->num, &(p + i)->score[0], &(p + i)->score[1]);
//  }
//
//  // 打印学生信息
//  printf("\n学生信息如下：\n");
//  printf("姓名\t学号\t成绩1\t成绩2\n");
//  printf("--------------------------------\n");
//  for (int i = 0; i < 2; i++)
//  {
//    //printf("%s\t%d\t%.1f\t%.1f\n",
//    //  arr[i].name,
//    //  arr[i].num,
//    //  arr[i].score[0],
//    //  arr[i].score[1]);
//    printf("%s\t%d\t%.1f\t%.1f\n",
//      (p + i)->name,
//      (p + i)->num,
//      (p + i)->score[0],
//      (p + i)->score[1]);
//
//  }
//
//  return 0;
//}
