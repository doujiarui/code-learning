#include "test.h"   

MyString::MyString(const char Str[])   
{   
    int length = strlen(Str);   
    m_data = new char[length+1];   
    strcpy(m_data,Str);   
}
MyString MyString::operator+(const MyString& RightString)   
{      
	int len1=strlen(this->m_data);        
	int len2=strlen(RightString.m_data);     
	MyString str(new char[len1+len2+1]);     
	strcpy(str.m_data,this->m_data);   
	strcat(str.m_data,RightString.m_data);  
	return str;  
}   
void MyString::set_str(const char Str[])
{    
	int length = strlen(Str);   
    m_data = new char[length+1];   
    strcpy(m_data,Str); 
}   
int MyString::get_length()   
{   
    return strlen(this->m_data);   
}   
