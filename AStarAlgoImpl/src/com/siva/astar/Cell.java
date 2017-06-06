package com.siva.astar;

public class Cell {
	// Keeping these members are default for AStarAlgo to access
	int heuristicCost = 0; //Heuristic cost
    int finalCost = 0; //G+H
    int i, j;
    int cost;
    Cell parent; 
    
    Cell(int i, int j, int cost){
        this.i = i;
        this.j = j;
        this.cost = cost;
    }
    
    @Override
    public String toString(){
        return "["+this.i+", "+this.j+"]";
    }
}
