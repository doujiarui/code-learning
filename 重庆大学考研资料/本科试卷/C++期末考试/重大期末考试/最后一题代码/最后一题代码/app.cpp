#include "test.h"
int main()   
{      
    MyString s1,s2,s3;
    s1.set_str("This is my first String");
	s2=MyString("This is my second string");
	s3=s1+s2;
 	cout<<s3.get_length()<<endl;
 	cout<<s3<<endl;
    return 0;   
}  
