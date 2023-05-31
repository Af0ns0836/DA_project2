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
        cout << "2.Backtracking" << endl;
        cout << "Enter q to terminate the program or to return to a previous menu" << endl;
        cout << "Enter the respective number: ";
        //cin >> option;
        if(option == 'q') end = true;
        option = '4';
        switch (option) {
            case '1':
                menu1(&tsp);
                break;
            case '2':{
                string filename ="shipping";
                tsp.readSmallDataSet(filename);
                cout << tsp.getPaths();
                end = true;
                break;
            }
            case '3':
                menu3(&tsp);
                break;
            case '4':
                string filename = "graph1";
                tsp.readMediumDataSet("edges_25");
                //tsp.readBigDataSet(filename);
                /*double ans;
                tsp.simulatedAnnealing(ans);*/
                tsp.printMST(tsp.MST());
                end = true;
                break;
        }
    }
}
void Menu::menu1(TSP *tsp) {
    bool end = false;
    char option;
    string filename;
    while (!end) {
        cout << "#############################" << endl;
        cout << "#      Select dataset       #" << endl;
        cout << "#############################" << endl;
        cout << "1->The small graphs are the following: shipping, stadiums, tourism." << endl;
        cout << "2->The real world graphs are the following: graph1, graph2, graph3." << endl;
        cout << "Choose the type of graph( 1 or 2 ): ";
        cin >> option;
        switch (option) {
            case '1':
                cout << "Write the name of the file: ";
                cin >> filename;
                tsp->readSmallDataSet(filename);
                end = true;
                break;
            case '2':
                cout << "Write the name of the file: ";
                cin >> filename;
                tsp->readBigDataSet(filename);
                end = true;
                break;
            default:
                end = true;
                break;
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