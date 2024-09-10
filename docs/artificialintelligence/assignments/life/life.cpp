#include <iostream>

using namespace std;

struct Matrix {
  int rows, columns, turns;
};

int main() {
  int rows, columns, turns;
  cin >> rows >> columns >> turns;
  cout << "Rows: " << rows << endl;
  cout << "Columns: " << columns << endl;
  cout << "Turns: " << turns << endl;
}
