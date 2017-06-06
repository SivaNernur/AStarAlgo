package com.siva.astar;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.RandomAccessFile;
import java.io.Reader;
import java.nio.charset.Charset;
import java.util.*;

public class AStarAlgo {

	public static final String TERRAIN_INFO = "large_map.txt";
	public static final int STARTPOINT_COST = 1;
	public static final int ENDPOINT_COST = 1;
	public static final int FLATLAND_COST = 1;
	public static final int FOREST_COST = 2;
	public static final int MOUNTAIN_COST = 3;

	static Cell[][] grid;
	static PriorityQueue<Cell> open;

	static boolean closed[][];
	static int startI, startJ;
	static int endI, endJ;
	static boolean endPointSet = false;
	static boolean startPointSet = false;
	static int terrainSquareDimension = 0;

	public static void setBlocked(int i, int j) {
		grid[i][j] = null;
	}

	public static void setStartCell(int i, int j) {
		startI = i;
		startJ = j;
		startPointSet = true;
	}

	public static void setEndCell(int i, int j) {
		endI = i;
		endJ = j;
		endPointSet = true;
	}

	private static void checkAndUpdateCost(Cell current, Cell t) {
		if (t == null || closed[t.i][t.j])
			return;

		// While parsing, for the cells which come before end point identification,
		// heuristicCost could not be calculated. Calculate now while movement
		if (t.heuristicCost == 0)
		{
			t.heuristicCost = Math.abs(t.i-endI)+Math.abs(t.j-endJ);
		}

		int t_final_cost = current.finalCost + t.heuristicCost + t.cost;

		boolean inOpen = open.contains(t);
		if (!inOpen || t_final_cost < t.finalCost) {
			t.finalCost = t_final_cost;
			t.parent = current;
			if (!inOpen)
				open.add(t);
		}
	}

	private static void AStar() {

		// add the start location to open list.
		open.add(grid[startI][startJ]);

		Cell current;

		while (true) {
			current = open.poll();
			if (current == null)
				break;
			closed[current.i][current.j] = true;

			// If we reached the end, nothing to do. Return.
			if (current.equals(grid[endI][endJ])) {
				return;
			}

			Cell t;
			if (current.i - 1 >= 0) {
				t = grid[current.i - 1][current.j];
				checkAndUpdateCost(current, t);

				if (current.j - 1 >= 0) {
					t = grid[current.i - 1][current.j - 1];
					checkAndUpdateCost(current, t);
				}

				if (current.j + 1 < grid[0].length) {
					t = grid[current.i - 1][current.j + 1];
					checkAndUpdateCost(current, t);
				}
			}

			if (current.j - 1 >= 0) {
				t = grid[current.i][current.j - 1];
				checkAndUpdateCost(current, t);
			}

			if (current.j + 1 < grid[0].length) {
				t = grid[current.i][current.j + 1];
				checkAndUpdateCost(current, t);
			}

			if (current.i + 1 < grid.length) {
				t = grid[current.i + 1][current.j];
				checkAndUpdateCost(current, t);

				if (current.j - 1 >= 0) {
					t = grid[current.i + 1][current.j - 1];
					checkAndUpdateCost(current, t);
				}

				if (current.j + 1 < grid[0].length) {
					t = grid[current.i + 1][current.j + 1];
					checkAndUpdateCost(current, t);
				}
			}
		}
	} // AStar method ends

	public static void identifyShortestPath(int tiles, int si, int sj, int ei, int ej) {
		if (!startPointSet || !endPointSet)
			return; // Invalid Information. Missing Source or destination. Exception can be thrown

		int row = tiles;
		int col = tiles;
		closed = new boolean[row][col];
		open = new PriorityQueue<>((Object o1, Object o2) -> {
			Cell c1 = (Cell) o1;
			Cell c2 = (Cell) o2;

			return c1.finalCost < c2.finalCost ? -1 : c1.finalCost > c2.finalCost ? 1 : 0;
		});

		// Display initial map before tracking path. For Input terrain display
		System.out.println("\n Grid: ");
		for (int i = 0; i < row; ++i) {
			for (int j = 0; j < col; ++j) {
				if (i == si && j == sj)
					System.out.print("SO  "); // Source
				else if (i == ei && j == ej)
					System.out.print("DE  "); // Destination
				else if (grid[i][j] != null)
					System.out.printf("%-3d ", 0);
				else
					System.out.print("BL  ");
			}
			System.out.println();
		}
		System.out.println();

		AStar();

		System.out.println("\nScores for cells: ");
		for (int i = 0; i < row; ++i) {
			for (int j = 0; j < row; ++j) {
				if (grid[i][j] != null)
					System.out.printf("%-3d ", grid[i][j].finalCost);
				else
					System.out.print("BL  ");
			}
			System.out.println();
		}
		System.out.println();

	} // Test Method ends

	private static int setTerrainSize(File file, Charset encoding) {
		int rows = 0;
		try (InputStream in = new FileInputStream(file);
				Reader reader = new InputStreamReader(in, encoding);
				BufferedReader buffer = new BufferedReader(reader)) {
			rows = getLinesCount(buffer);
			initializeTerrainSpace(rows);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return rows;
	}

	private static void prepareTerrainFromInfo(File file, Charset encoding) {
		try (InputStream in = new FileInputStream(file);
				Reader reader = new InputStreamReader(in, encoding);
				BufferedReader buffer = new BufferedReader(reader)) {
			parseTerrainStepByStep(buffer);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static int getLinesCount(BufferedReader reader) {
		int lineCount = 0;
		try {
			while ((reader.readLine()) != null) {
				lineCount++;
				// System.out.println("Lines = " + lineCount);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}

		return lineCount;
	}

	private static void initializeTerrainSpace(int tiles) {
		System.out.println("Terrain Square Dimension : "+tiles);
		grid = new Cell[tiles][tiles];
	}

	private static void parseTerrainStepByStep(Reader reader) {
		int row = 0;
		int column = 0;
		int nextChar = 0;
		char charRead;
		char prevChar = '\0';

		try {
			while ((nextChar = (char)reader.read()) != -1) {
				//System.out.println("next Char is "+nextChar);
				charRead = (char)nextChar;
				switch (charRead) {
				case '~' : // Water
				{
					grid[row][column] = null;
					column++;
					break;
				}
				case '@' : // Flatland StartPoint
				{
					grid[row][column] = new Cell(row, column, STARTPOINT_COST);
					// System.out.println("Char "+nextChar+ " START Point : Row "+row+" Col "+column);
					if (endPointSet)
					{
						grid[row][column].heuristicCost = Math.abs(row - endI) + Math.abs(column - endJ);
					}
					grid[row][column].finalCost = 0;
					setStartCell(row, column);
					column++;
					break;
				}
				case 'X' : // Flatland EndPoint
				{
					grid[row][column] = new Cell(row, column, ENDPOINT_COST);
					if (endPointSet)
					{
						grid[row][column].heuristicCost = Math.abs(row - endI) + Math.abs(column - endJ);
					}
					setEndCell(row, column);
					// System.out.println("Char "+nextChar+ " END Point : Row "+row+" Column "+column);
					column++;
					break;
				}
				case '.' : // Flatland
				{
					//System.out.println("Char "+nextChar+ " at Row "+row+" Col "+column);
					grid[row][column] = new Cell(row, column, FLATLAND_COST);
					if (endPointSet)
					{
						grid[row][column].heuristicCost = Math.abs(row - endI) + Math.abs(column - endJ);
					}
					column++;
					break;
				}
				case '*': // Forest
				{
					// System.out.println("Char "+nextChar+ " at Row "+row+" Col "+column);
					grid[row][column] = new Cell(row, column, FOREST_COST);
					if (endPointSet)
					{
						grid[row][column].heuristicCost = Math.abs(row - endI) + Math.abs(column - endJ);
					}
					column++;
					break;
				}
				case '^' : // Mountain
				{
					// System.out.println("Char "+nextChar+ " at Row "+row+" Col "+column);
					grid[row][column] = new Cell(row, column, MOUNTAIN_COST);
					if (endPointSet)
					{
						grid[row][column].heuristicCost = Math.abs(row - endI) + Math.abs(column - endJ);
					}
					column++;
					break;
				}
				case '\r': {
					column = 0;
					row++;
					// System.out.println("CarReturn at Row "+row+" Col "+column);
					break;
				}
				case '\n': {
					column = 0;
					if (prevChar == '\r') {
						// The current line is terminated by a carriage return immediately followed by a line feed.
						// The line has already been counted.
						// System.out.println("NO INCR : CarReturn at Row "+row+" Col "+column);
					} else {
						// The current line is terminated by a line feed.
						row++;
						// System.out.println("NewLine at Row "+row+" Col "+column);
					}
					break;
				}
				default:
				{
					// System.out.println("invalid character = "+charRead);
					return;
				}
				}
				prevChar = charRead;
			} // while ends
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}

	private static void markShortestPath() {
		int posToMark = 0;
		int lengthOfLine = terrainSquareDimension+2; // rows+'/r'+'\n'

		try {
			RandomAccessFile ra = new RandomAccessFile(TERRAIN_INFO, "rw");

			if (closed[endI][endJ]) {
				// Trace back the path
				System.out.println("Path: ");

				// Mark the path from First Move to Destination
				Cell current = grid[endI][endJ];
				System.out.print(current);
				while (current.parent != null) {
					/* Where terrainSquareDimension gives no of columns, at end of
					 * each line, we have extra two characters '\r' & '\n'
					 * So, calculate row*(squareDimension +1[for \r] and +1[for \n]) 
					 */
					posToMark = ((current.i*(lengthOfLine))+current.j);
					ra.seek(posToMark);
					ra.writeByte('#');
					System.out.print(" -> " + current.parent);
					current = current.parent;
				}

				// Mark the Start Location explicitly
				current = grid[startI][startJ];
				posToMark = ((current.i*(lengthOfLine))+current.j);
				ra.seek(posToMark);
				ra.writeByte('#');
			} 
			else
			{
				System.out.println("No possible path");
			}

			ra.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public static void main(String[] args) {

		Charset encoding = Charset.defaultCharset();
		File file = new File(TERRAIN_INFO);

		terrainSquareDimension = setTerrainSize(file, encoding);
		System.out.println("Size of terrain = "+terrainSquareDimension);

		System.out.println("Prepare Terrain");
		prepareTerrainFromInfo(file, encoding);

		if (terrainSquareDimension == 0)
			return;

		System.out.println("Mark Shortest Path");
		identifyShortestPath(terrainSquareDimension, startI, startJ, endI, endJ);

		markShortestPath();
	}
}
