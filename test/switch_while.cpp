#include <iostream>
using namespace std;

int main() {
    int a=20, b=10;
    int c=10, d=20;
    cout << "start" << endl;
    while(true) {
        bool flag = false;
        char key;
        cin >> key;
        switch(key){
            case 'j':
                while(flag == false) {
                    if (a != b || c != d){
                        a < b ? ++a : --a;
                        c < d ? ++c : --c;
                    }else {
                        flag = true;
                    }
                    cout << "a: " << a << endl;
                    cout << "c: " << c << endl;
                }
                break;
            default:
                cout << "default" << endl;
                break;
        }
    }
}
