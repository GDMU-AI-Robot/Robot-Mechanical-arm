//#define _CRT_SECURE_NO_WARNINGS
//#include <stdio.h>
//#include <stdlib.h>
//
////int main()
////{
////  FILE* pf = fopen("djj.txt", "r");
////  if (pf == NULL)
////  {
////    printf("�ļ���ʧ��");
////    return 1;
////  }
////
////  // ���ļ���ʹ����ȷ��fgetc����
////  int ch = fgetc(pf);
////  if (ch != EOF)  // ����ȡ�Ƿ�ɹ�
////  {
////    printf("��ȡ�����ַ�: %c\n", ch);
////  }
////  else
////  {
////    printf("��ȡ�ļ�ʧ�ܻ��ļ�Ϊ��\n");
////  }
////  // �ر��ļ�
////  fclose(pf);
////  pf = NULL;  // ����Ұָ��
////  return 0;
//// }
//
//int main()
//{
//  char ch;
//  FILE* pf = fopen("djj.txt", "w");
//  if (pf == NULL)
//  {
//    printf("�ļ���ʧ��!\n");
//  }
//
//  //ch = getchar();
//  //while (ch != '#')
//  //{
//  //  fputc(ch, pf);
//  //  putchar(ch);
//  //  ch = getchar();
//  //}
//
//  //�Ӽ������������ݣ�Ȼ���ȡ���ݣ�д���ļ��ʹ�ӡ
//  while ((ch = getchar()) != '#')
//  {
//    fputc(ch, pf);
//    putchar(ch);
//  }
//
//
//  ////д����
//  //fputc('D', pf);
//  //fputc('J', pf);
//  //fputc('J', pf);
//
//  //�ر��ļ�
//  fclose(pf);
//  pf = NULL;
//}