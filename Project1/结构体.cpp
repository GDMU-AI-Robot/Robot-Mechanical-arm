//#define _CRT_SECURE_NO_WARNINGS
//#include <stdio.h>
//
//int main()
//{
//  struct stu
//  {
//    char name[20];
//    int num;
//    float score[2];  // �洢���ųɼ�
//  };
//  struct stu* p;
//  struct stu arr[2];  // ���Դ洢2��ѧ����Ϣ
//  p = arr;  // ��ָ��pָ������arr���׵�ַ
//
//  for (int i = 0; i < 2; i++)
//  {
//    printf("�������%d��ѧ������Ϣ(���� ѧ�� �ɼ�1 �ɼ�2): ", i + 1);
//    // ʹ��scanf_sʱ�������ַ�����Ҫָ����������С
//    //scanf_s("%s %d %f %f", arr[i].name, sizeof(arr[i].name),
//    //  &arr[i].num, &arr[i].score[0], &arr[i].score[1]);
//    scanf("%s %d %f %f", (p + i)->name,
//      &(p + i)->num, &(p + i)->score[0], &(p + i)->score[1]);
//  }
//
//  // ��ӡѧ����Ϣ
//  printf("\nѧ����Ϣ���£�\n");
//  printf("����\tѧ��\t�ɼ�1\t�ɼ�2\n");
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
