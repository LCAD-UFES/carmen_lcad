
#include <carmen/subdir_1.hpp>
extern "C" {
#include <carmen/subdir_2.h> 
}

#include <iostream>

using namespace std;

int main()
{
	subdir_1();
	subdir_2();

	cout << "main" << endl;
	return 0;
}

