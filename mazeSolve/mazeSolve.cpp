#include <iostream>

using namespace std;

int LEFT = 1;
int RIGHT = -1;
int FORWARD = 2;

void solve(int moves[50]) {

    for( int i = 0; i < 50; i++){
      if(moves[i] == LEFT && moves[i+1] == LEFT){
        int temp = 0;
        i += 2;
        while(moves[i + temp] == FORWARD || (moves[i+temp] == -moves[i- temp - 3])){
          temp++;
        }
        cout << i - temp - 3 << endl;
        if(moves[i - temp - 3] == RIGHT && moves[i + temp ] == RIGHT)
        {
            // moves[i - temp - 3] = FORWARD;
            // moves[i + temp + 1] = 0;
            moves[i - temp - 3] = 0;
            moves[i + temp] = 0;
        }
        for (int j = i - temp - 2; j < i + temp; j++) {
          moves[j] = 0;
        }
        i += temp;
      }
    }
}

void removeZeros(int moves[50]) {
    int temp[50] = {};
    int j = 0;
    for (int i = 0; i < 50; i++) {
        if (moves[i] != 0) {
            temp[j] = moves[i];
            j++;
        }
    }
    for(int i = 0; i < 50; i++) {
        moves[i] = temp[i];
    }
}


int main()
{
    //int moves[50] = {FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT};
    //int moves[50] = {FORWARD, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD};
    // int moves[50] = {FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, 
    // FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, LEFT, FORWARD, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD};
    int moves[50] = {FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, LEFT, FORWARD, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD};
    solve(moves);
    removeZeros(moves);
    solve(moves);
    //removeZeros(moves);
    for( int i = 0; i < 50; i++){
        cout << moves[i] << ' ';
    }
    return 0;
}

