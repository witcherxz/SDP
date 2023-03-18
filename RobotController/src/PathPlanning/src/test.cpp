#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sstream>
#include <conio.h>


#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67
using namespace std;

int main(int argc, char **argv){
    int c;
	while(1){
       
		switch((c=getch())){
	    case KEY_UP:
            	cout << std::endl << "Up" << std::endl;
				break;
        case KEY_DOWN:
            	cout << std::endl << "Down" << std::endl;
            	break;
        case KEY_LEFT:
            	cout << std::endl << "Left" << std::endl;  // key left
            	break;
        case KEY_RIGHT:
            	cout <<std:: endl << "Right" << std::endl;  // key right
		        break;
		}
	//cout << std::endl << c << std::endl;  // not arrow 
	}
	return 0;
}
