#include <iostream>

using namespace std;

int treasureMap[6][6] = {{}};
int compass = 1; 
int main(){
    int LEFT = 1;
    int RIGHT = -1;
    int FORWARD = 2;
    int moves[50] = {FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, 
        FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, LEFT, FORWARD, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD}; //instructions that the bot would have gotten if it explored properly
    int x = 0;
    int y = 0;

    for(int i = 0; i < 50; i++){
        if(moves[i] == FORWARD){
            treasureMap[x][y] = compass; //whenever the bot physically moves to a new square it will update the direction it took in the 2d array 
            switch(compass){    

                case(1): //if facing North when moving y increments
                y++;
                break;
                case(2): //if facing East when moving x increments
                x++;
                break;
                case(3): //if facing South when moving y decrements
                y--;
                break;
                case(4): //if facing West when moving x decrements
                x--;
                break;
            }

        }else if(moves[i] == RIGHT){ //The direction it faces is updated whenever it turns. Right turn is positive left turn is negative
            compass = ((compass) % 4) + 1; 
        }else if(moves[i] == LEFT){
            if(compass ==1){
                compass = 4;
            }else{
                compass--;
            }
        }

    }
    treasureMap[x][y] = 5;
      
    for(int j = 5; j >=0 ; j--){ //indexed so that north is up when printing
        for(int i = 0; i < 6; i++){
            switch(treasureMap[i][j]){ //formatted for easy understanding of print
                case(1):
                    cout << "↑ ";
                    break;
                case(2):
                    cout << "→ ";
                    break;
                case(3):
                    cout << "↓ ";
                    break;
                case(4): 
                    cout << "← ";
                    break;
                case(5): 
                    cout << "X ";
                    break;
                default:
                    cout << "  ";


            }
            //cout << treasureMap[i][j] << ' ';
        }
        cout << endl;
    }


    return 0;
}