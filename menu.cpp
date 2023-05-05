//
// Created by dias4 on 05/05/2023.
//

#include "menu.h"
#include "TSP.h"

using namespace std;

void Menu::menu(){
    TSP tsp = TSP();

    bool end = false;
    char option;
    while(!end){
        cout << "#############################" << endl;
        cout << "#         MAIN MENU         #" << endl;
        cout << "#############################" << endl;
        cout << "1.Choose the dataset" << endl;
        cout << "Enter q to terminate the program or to return to a previous menu" << endl;
        cout << "Enter the respective number: ";
        cin >> option;
        if(option == 'q') end = true;
        switch (option) {
            case '1':
                menu1(&tsp);
                break;
            case '2':
                menu2(&tsp);
                break;
            case '3':
                menu3(&tsp);
                break;
            case '4':
                menu4(&tsp);
                break;
        }
    }
}
void Menu::menu1(TSP *tsp) {
    bool end = false;
    char option;
    while (!end) {
        cout << "#############################" << endl;
        cout << "#      Select dataset       #" << endl;
        cout << "#############################" << endl;
        cout << "1.Shipping(Toy-Graph)" << endl;
        cout << "2.Stadiums(Toy-Graph)" << endl;
        cout << "3.Tourism(Toy-Graph)" << endl;
        cout << "4.graph1(real-world)" << endl;
        cout << "5.graph2(real-world)" << endl;
        cout << "6.graph3(real-world)" << endl;
        cout << "Enter the respective number: ";
        cin >> option;
        switch (option) {
            case '1':
                tsp->readSmallDataSet("shipping.csv"); break;
            /*case 2:
                tsp->readSmallDataSet("stadiums.csv"); break;
            case 3:
                tsp->readSmallDataSet("tourism.csv"); break;
            case 4:
                tsp->readSmallDataSet("shipping.csv"); break;
            case 5:
                tsp->readSmallDataSet("shipping.csv"); break;
            case 6:
                tsp->readSmallDataSet("shipping.csv"); break;*/
            default:
                end = true;
                break;
        }
    }
}

void Menu::menu2(TSP *tsp) {
    bool end = false;
    char option;
    while(!end){
        cout << "#############################" << endl;
        cout << "#  Backtracking algorithm   #" << endl;
        cout << "#############################" << endl;
        cout << "1.Choose the dataset" << endl;
        cout << "Enter q to terminate the program or to return to a previous menu" << endl;
        cout << "Enter the respective number: ";
        cin >> option;
        if(option == 'q') end = true;
        else if(option == '1'){

        }
    }
}

void Menu::menu3(TSP *tsp) {
    bool end = false;
    char option;
    while(!end){
        cout << "######################################" << endl;
        cout << "# Triangular Approximation Heuristic #" << endl;
        cout << "######################################" << endl;
        cout << "1.Choose the dataset" << endl;
        cout << "Enter q to terminate the program or to return to a previous menu" << endl;
        cout << "Enter the respective number: ";
        cin >> option;
        if(option == 'q') end = true;
        else if(option == '1'){

        }
    }
}

void Menu::menu4(TSP *tsp) {
    bool end = false;
    char option;
    while(!end){
        cout << "############################" << endl;
        cout << "#     Other Heuristics     #" << endl;
        cout << "############################" << endl;
        cout << "1.Choose the dataset" << endl;
        cout << "Enter q to terminate the program or to return to a previous menu" << endl;
        cout << "Enter the respective number: ";
        cin >> option;
        if(option == 'q') end = true;
        else if(option == '1'){

        }
    }
}