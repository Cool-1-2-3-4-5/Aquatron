#include "vex.h"
using namespace vex;

// Main robot routine: explore the field, locate each plant color in sequence,
// water it, and return to the starting position before continuing.
int main()
{
    brain Brain;
    Drivetrain drive(PORT7, PORT12, PORT1, PORT6, PORT9);
    Pump PumpMotor(PORT10);

    // Set up the state used for the plant-search and watering loop.
    drive.setGrid(3, 3);
    int old_grid[3][3] = {};
    int grid[3][3] = {};
    bool visit_Array[3][3] = {};
    int current_x = 0;
    int current_y = 0;
    bool finalcheck = true;
    bool verify[3][3] = {};
    int movement[50] = {0};
    int dead[50] = {0};
    int going[50] = {0};
    int coming[50] = {0};
    int x_pos = 0;
    int y_pos = 0;
    int new_x = 0;
    int new_y = 0;
    int cur_x = 0;
    int cur_y = 0;
    int verifycnt = 0;
    int numcnt = 0;
    int wanted_x = 0;
    int wanted_y = 0;
    int secondcnt = 0;
    int finalcnt = 0;
    int deadcnt = 0;
    int water_time = 0;
    int plant_color = 0;
    int going_index = 0;
    int index = 0;
    bool check = true;
    int direction = 0;
    // Wait for the start signal and begin the autonomous routine.
    drive.touchandgo();
    drive.dfs(old_grid, current_x, current_y, visit_Array);
    drive.array_changer(old_grid,grid);
    Brain.Screen.clearScreen();
    wait(2, seconds);
    for (int color_to_find = 1; color_to_find <= 4; color_to_find++)
    {
        // Reset state for the next plant color before searching again.
        check = true;
        finalcheck = true;
        numcnt = 0;
        x_pos = 0;
        y_pos = 0;
        new_x = 0;
        new_y = 0;
        cur_x = 0;
        cur_y = 0;
        verifycnt = 0;
        direction = 0;
        secondcnt = 0;
        deadcnt = 0;
        for (int i = 0; i < 50; i++)
        {
            movement[i] = 0;
            dead[i] = 0;
            going[i] = 0;
            coming[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                verify[i][j] = false;
            }
        }
        // Locate the target color on the known grid and confirm it exists.
        drive.index_finder(wanted_x, wanted_y, grid, color_to_find, check);
        Brain.Screen.printAt(10, 50, "Found at [%d][%d]", wanted_x, wanted_y);
        wait(1, seconds);
        Brain.Screen.clearScreen();
        // Only continue if the color was actually found on the map.
        if (wanted_x >= 0 && wanted_y >= 0 && wanted_x < 3 && wanted_y < 3 && check)
        {
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 30, "Searching color: %d", color_to_find);
            Brain.Screen.printAt(10, 50, "Found at [%d][%d]", wanted_x, wanted_y);
            wait(2, seconds);
            Brain.Screen.clearScreen();
            // Plan the shortest valid route from the robot to the target plant.
            drive.mapping(grid, numcnt, finalcheck, x_pos, y_pos, new_x, new_y, verify, verifycnt, movement, dead, wanted_x, wanted_y);
            // Count the number of movement steps in the planned path.
            while (movement[secondcnt] != 0)
            {
                secondcnt++;
            }
            // Count dead-end moves so they can be removed from the final route.
            for (int i = 0; i < secondcnt; i++)
            {
                if (dead[i] == 1)
                {
                    deadcnt++;
                }  
            }
            // Keep only the useful route, leaving out useless backtracking.
            finalcnt = secondcnt - deadcnt;
            going_index = 0;
            for (int j = 0; j < secondcnt; j++)
            {
                if (dead[j] != 1)
                {
                    going[going_index] = movement[j];
                    going_index++;
                }
            }
            // Build the return path by reversing the travel directions.
            index = 0;
            for (int i = finalcnt - 1; i >= 0; i--)
            {
                direction = going[i];
                if (direction == 1)
                {
                    coming[index] = 2;
                }
                else if (direction == 2)
                {
                    coming[index] = 1;
                }
                else if (direction == 3)
                {
                    coming[index] = 4;
                }
                else if (direction == 4)
                {
                    coming[index] = 3;
                }
                else
                {
                    coming[index] = 0;
                }
                index++;
            }

            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 50, "Path calculated!");
            Brain.Screen.printAt(10, 70, "Steps: %d", finalcnt);
            
            wait(2, seconds);
            Brain.Screen.clearScreen();
            // Show the intended path one step at a time for debugging.
            for (int i = 0; i < finalcnt; i++)
            {
                Brain.Screen.printAt(10, 50, "This is path: %d", going[i]);
                wait(1, seconds);
                Brain.Screen.clearScreen();
            }            
            // Drive to the target plant following the calculated route.
            drive.GoToPos(going, finalcnt);
            // Look up the plant color and convert it to a watering duration.
            plant_color = grid[wanted_x][wanted_y];
            water_time = drive.colourtotime(plant_color);
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 50, "Watering plant...");
            Brain.Screen.printAt(10, 70, "Color: %d Time: %d", plant_color, water_time);
            // Water the plant for the correct amount of time.
            PumpMotor.PourWater(water_time);
            wait(1, seconds);
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 50, "Returning home...");
            wait(1, seconds);
            // Return to the start and reset the robot heading.
            drive.comeHome(coming, finalcnt);
            drive.PIDturn(0);
        }
        else
        {
            // The target color was not detected in the grid.
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 50, "Color %d not Found!", color_to_find);
            wait(1, seconds);
        }
    }
    // Every planned plant has been watered; finish the run.
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 50, "All plants watered!");
    Brain.Screen.printAt(10, 50, "for today!");
    wait(2, seconds);
    Brain.programStop();   
}