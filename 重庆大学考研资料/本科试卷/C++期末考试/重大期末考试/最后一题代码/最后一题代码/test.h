#include "iostream"  
#include<String.h>  
using namespace std;   
class MyString   
{   
public:                                     
    MyString (const char Str[]="");  
    void set_str(const char Str[]);
	int get_length();
    MyString operator+(const MyString& RightString); 
    friend ostream & MyString::operator<<(ostream& os,MyString& str)   
    {   
        os<<str.m_data;   
        return os;   
    }    
    char* m_data;                                
};  
