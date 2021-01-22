#include <iostream>
#include <stdio.h>

void foo() {
	std::cout << "foo is called" << std::endl;
}

void bar() {
	printf("bar is called\n");
}

// 主函数入口
int main() {
	std::cout << "hello world" << std::endl;	
	foo();
	bar();

	std::cout << "hello world namespace std\n";
	std::cout << "second push\n";

	return 0;
}
