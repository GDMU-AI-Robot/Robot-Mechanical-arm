//#include <iostream>
//using namespace std;
//
//int main()
//{
//  cout << "hello C++" << endl;
//  return 0;
//}

//#include <stdio.h>
//int main()
//{
//  return 0;
//}
//int main()
//{
//	int Aa = 2;
//	switch (Aa)
//	{
//	case 1:
//		printf("111");
//	case 2:
//		printf("222"); break;
//	default:break;
//	}
//	return 0;
//}

//int main()
//{
//  int a, b, c;
//  double total=0;
//  for (int i = 1; i <= 20; i++)
//  {
//    double num = 1;
//    for (int j = i; j > 0; j--)
//    {
//      num = num * j;
//    }
//    total =total+ num;
//  }
//  printf("%lf", total);
//  return 0;
//}

//void max(int add,int n)
//{
//  int i = 0;
//  for ( i = 0; i < n; i++)
//  {
//    printf("%d", add[i]);
//  }
//}



//void max(int arr[])
//{
//  int i = 0;
//  for (i = 0; i < 10; i++)
//  {
//    printf("%d", arr[i]);
//  }
//}

//void Move(int* p, int sz)//1,2,3,4,5,6,7,8,9,10 °´ÕÕÏà·´Ë³ÐòÅÅÐò
//{
//  int* left = p; 
//  int* right = p + sz - 1; 
//  while (left < right)
//  {
//    int tmp = *left; 
//    *left = *right; 
//    *right = tmp; 
//    left++; 
//    right--;
//  }
//}

/*
int main()//Ã°ÅÝÅÅÐò Ö¸Õë°æ±¾
{
  void Paixv(int* p);
  int arr[10] = {1,4,3,2,5,6,7,8,9,10};
  int* p, i;
  p = arr;
  for (int i = 0; i < 10; i++)
  {
    printf("%d\t", *p++);
  }
  printf("\n");
  p = arr;
  Paixv(p);


  p = arr;
  for (int i = 0; i < 10; i++)
  {
    printf("%d\t", *p++);
  }
  printf("\n");

  return 0;
}

void Paixv(int* p)
{
  int i;
  for (i = 0; i < 9; i++)
  {
    for (int j = 0; j < 9 - i; j++)
    {
      if (*(p+j) < *(p+j+1))
      {
        int tmp;
        tmp = *(p + j);
        *(p + j) = *(p + j + 1);
        *(p + j + 1) = tmp;
      }
    }
  }
}
*/


//int main()//Ã°ÅÝÅÅÐò Êý×é°æ±¾
//{
//  int arr[10] = { 1,4,2,3,5,6,7,8,9,10 };
//  int i = 0;
//
//  for (int i = 0; i < 10; i++)
//  {
//    printf("%d\t", arr[i]);
//  }
//  printf("\n");
//  for (i = 0; i < 9; i++)
//  {
//    int j = 0;
//    for ( j = 0; j < 9-i; j++)
//    {
//      if (arr[j] < arr[j + 1])
//      {
//        int temp;
//        temp = arr[j];
//        arr[j] = arr[j + 1];
//        arr[j+1] = temp;
//      }
//    }
//  }
//  for (int i = 0; i < 10; i++)
//  {
//    printf("%d\t", arr[i]);
//  }
//  printf("\n");
//
//  return 0;
//}