//#define _CRT_SECURE_NO_WARNINGS
//#include <stdio.h>
//#include <stdlib.h>
//
////int main()
////{
////  FILE* pf = fopen("djj.txt", "r");
////  if (pf == NULL)
////  {
////    printf("文件打开失败");
////    return 1;
////  }
////
////  // 读文件，使用正确的fgetc函数
////  int ch = fgetc(pf);
////  if (ch != EOF)  // 检查读取是否成功
////  {
////    printf("读取到的字符: %c\n", ch);
////  }
////  else
////  {
////    printf("读取文件失败或文件为空\n");
////  }
////  // 关闭文件
////  fclose(pf);
////  pf = NULL;  // 避免野指针
////  return 0;
//// }
//
//int main()
//{
//  char ch;
//  FILE* pf = fopen("djj.txt", "w");
//  if (pf == NULL)
//  {
//    printf("文件打开失败!\n");
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
//  //从键盘中输入数据，然后获取数据，写出文件和打印
//  while ((ch = getchar()) != '#')
//  {
//    fputc(ch, pf);
//    putchar(ch);
//  }
//
//
//  ////写数据
//  //fputc('D', pf);
//  //fputc('J', pf);
//  //fputc('J', pf);
//
//  //关闭文件
//  fclose(pf);
//  pf = NULL;
//}