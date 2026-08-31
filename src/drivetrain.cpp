#include "drivetrain.hpp"

// Initialize the drivetrain, configure the motors, and calibrate the inertial sensor.
Drivetrain::Drivetrain(char left_Port, char right_Port, char distanceSensor_port, char colourSensor_port, char touchSensor_port)
    : left_(left_Port, false), right_(right_Port, true), DistanceSensor(distanceSensor_port), ColourSensor(colourSensor_port), TouchSensor(touchSensor_port)
{
    left_.setStopping(brakeType::hold);
    right_.setStopping(brakeType::hold);
    left_.setVelocity(0, percent);
    right_.setVelocity(0, percent);
    left_.setPosition(0, turns);
    right_.setPosition(0, turns);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono15);
    ColourSensor.setLight(ledState::on);
    ColourSensor.brightness(100);
    BrainInertial.calibrate();
    while (BrainInertial.isCalibrating())
    {
        wait(50, msec);
    }
    BrainInertial.setRotation(0, degrees);
    BrainInertial.setHeading(0, degrees);

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 50, "Drivetrain Initialized!");
    wait(1, seconds);
    Brain.Screen.clearScreen();
}
// Clean shutdown message when the class is destroyed.
Drivetrain::~Drivetrain()
{
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 50, "Code Complete!");
}

// Save the size of the grid so the robot can safely navigate between cells.
void Drivetrain::setGrid(int x, int y)
{
    grid_rows = x;
    grid_cols = y;
}

void Drivetrain::stop()
{
    left_.stop();
    right_.stop();
}
// Move the robot a target distance while correcting drift with a basic PID loop.
void Drivetrain::PIDmove(float distance, float kp, float ki, float kd)
{
    left_.setPosition(0, degrees);
    right_.setPosition(0, degrees);
    const float MAX_TIME = 5;
    const float position_tolerance = 0.1;
    float average_dist = 0;
    float derivate_difference;
    float cumerror = 0;
    float error = 0;
    float speed;
    float prev_error = 0;
    timeout.reset();
    left_.spin(forward);
    right_.spin(forward);
    while (timeout.time(seconds) < MAX_TIME)
    {
        average_dist = ((left_.position(degrees) + right_.position(degrees)) / 2) * 200.0 / 360;
        Brain.Screen.printAt(10, 50, "dist: %.1f", error);

        error = distance - average_dist;
        if (error < 0.0)
        {
            error = -error;
        }

        if (error <= position_tolerance)
        {
            break;
        }

        cumerror += distance - average_dist;
        derivate_difference = (distance - average_dist) - prev_error;
        speed = kp * (distance - average_dist) + ki * cumerror + kd * derivate_difference;

        left_.setVelocity(speed, percent);
        right_.setVelocity(speed, percent);
        prev_error = distance - average_dist;
        wait(20, msec);
    }

    left_.stop();
    right_.stop();
}
// Rotate to a target heading by comparing the inertial sensor against the desired angle.
void Drivetrain::PIDturn(float angle, float kp, float ki, float kd)
{
    const float MAX_TIME = 5;
    const float angle_tolerance = 1.0;
    float derivate_difference;
    float cumerror = 0;
    float error;
    float speed;
    float prev_error = 0;
    timeout.reset();
    left_.spin(forward);
    right_.spin(forward);

    if (angle > 180)
    {
        angle = angle - 360;
    }

    while (timeout.time(seconds) < MAX_TIME)
    {
        error = angle - BrainInertial.rotation();
        if (error < 0.0)
        {
            error = -error;
        }

        Brain.Screen.printAt(10, 50, "angle: %.1f", error);
        if (error <= angle_tolerance)
        {
            break;
        }

        cumerror += angle - BrainInertial.rotation();
        derivate_difference = (angle - BrainInertial.rotation()) - prev_error;
        speed = kp * (angle - BrainInertial.rotation()) + ki * cumerror + kd * derivate_difference;

        left_.setVelocity(speed, percent);
        right_.setVelocity(-speed, percent);
        prev_error = angle - BrainInertial.rotation();
        wait(20, msec);
    }

    left_.stop();
    right_.stop();
}
// Check whether there is a plant in front of the robot based on object distance.
bool Drivetrain::checkForPlant()
{
    Brain.Screen.printAt(10, 50, "DisPlant: %.1f", DistanceSensor.objectDistance(mm));
    wait(0.2, seconds);
    if (DistanceSensor.objectDistance(mm) < 200)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// Drive toward the detected plant, read its color, and return to the previous position.
int Drivetrain::moveToPlant()
{
    int colourVal = 0;
    float distance_initial = 0;
    float hue = 0;
    timeout.reset();
    distance_initial = DistanceSensor.objectDistance(mm);
    distance_initial = distance_initial - 20.0;
    PIDmove(distance_initial);
    hue = ColourSensor.hue();
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 50, "Hue: %.1f", hue);
    wait(0.2, seconds);
    if (hue >= 35 && hue <= 59)
    {
        colourVal = 1; // yellow
    }
    else if (hue >= 85 && hue <= 120)
    {
        colourVal = 2; // green
    }
    else if (hue >= 279 && hue <= 303)
    {
        colourVal = 3; // purple
    }
    else if (hue >= 340 || hue <= 24)
    {
        colourVal = 4; // orange/red
    }
    else
    {
        colourVal = 8; // unknown
    }
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 80, "Colour: %d", colourVal);
    wait(0.2, seconds);
    PIDmove(-1 * distance_initial);
    wait(0.2, seconds);
    return colourVal;
}
void Drivetrain::WateringPosition(float &distance_initial)
{
    distance_initial = DistanceSensor.objectDistance(mm);
    distance_initial = distance_initial - 30;
    PIDmove(distance_initial);
    wait(0.2, sec);
}

// Convert a plant color into the watering duration needed for that plant.
int Drivetrain::colourtotime(int colourValue)
{
    int timetowater = 0;
    if (colourValue == 1)
    {
        timetowater = 3;
    }
    if (colourValue == 2)
    {
        timetowater = 6;
    }
    if (colourValue == 3)
    {
        timetowater = 9;
    }
    if (colourValue == 4)
    {
        timetowater = 11;
    }
    return timetowater;
}
// Wait until the touch sensor is pressed and released before starting the run.
void Drivetrain::touchandgo()
{
    while (!TouchSensor.pressing())
    {
    }
    while (TouchSensor.pressing())
    {
    }
}

// Explore the grid recursively, marking visited cells and recording any plant locations found.
void Drivetrain::dfs(int grid[][3], int &current_x_pos, int &current_y_pos, bool visit_Array[][3])
{

    if (current_x_pos < 0 || current_x_pos >= grid_rows || current_y_pos < 0 || current_y_pos >= grid_cols)
    {
    }
    else
    {
        visit_Array[current_x_pos][current_y_pos] = true;
        int directions_change[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
        int posible_movement[4][2] = {};
        int cnt = 0;
        
        for (int i = 0; i < 4; i++)
        {
            int new_pos_x = current_x_pos + directions_change[i][0];
            int new_pos_y = current_y_pos + directions_change[i][1];

            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 30, "Checking dir %d", i);
            Brain.Screen.printAt(10, 50, "From [%d][%d]", current_x_pos, current_y_pos);
            Brain.Screen.printAt(10, 70, "To [%d][%d]", new_pos_x, new_pos_y);
            wait(0.2, seconds);
            if (new_pos_x < 0 || new_pos_x >= grid_rows || new_pos_y < 0 || new_pos_y >= grid_cols)
            {
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 50, "Out of bounds:");
                Brain.Screen.printAt(10, 70, "[%d][%d]", new_pos_x, new_pos_y);
                wait(0.2, seconds);
            }
            else if (visit_Array[new_pos_x][new_pos_y])
            {
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 50, "Already visited:");
                Brain.Screen.printAt(10, 70, "[%d][%d]", new_pos_x, new_pos_y);
                wait(0.2, seconds);
            }
            else
            {
                PIDturn(90 * i);
                if (checkForPlant())
                {
                    int colour = moveToPlant();
                    if (new_pos_x >= 0 && new_pos_x < grid_rows && new_pos_y >= 0 && new_pos_y < grid_cols)
                    {
                        grid[new_pos_x][new_pos_y] = colour;
                        visit_Array[new_pos_x][new_pos_y] = true;
                        Brain.Screen.printAt(10, 70, "Plant at [%d][%d]: %d", new_pos_x, new_pos_y, colour);
                        wait(0.2, seconds);
                        Brain.Screen.clearScreen();
                    }
                }
                else
                {
                    posible_movement[cnt][0] = new_pos_x;
                    posible_movement[cnt][1] = new_pos_y;
                    Brain.Screen.clearScreen();
                    Brain.Screen.printAt(10, 50, "Possible move %d:", cnt);
                    Brain.Screen.printAt(10, 70, "[%d][%d]", new_pos_x, new_pos_y);
                    wait(0.2, seconds);
                    cnt++;
                }
            }
        }
        for (int i = 0; i < cnt; i++)
        {
            int next_cell_x = posible_movement[i][0];
            int next_cell_y = posible_movement[i][1];

            if (!visit_Array[next_cell_x][next_cell_y])
            {
                int dx = next_cell_x - current_x_pos;
                int dy = next_cell_y - current_y_pos;
                int direction = 0;
                if (dy == -1)
                {
                    direction = 0; // Up
                }
                else if (dx == 1)
                {
                    direction = 1; // Right
                }
                else if (dy == 1)
                {
                    direction = 2; // Down
                }
                else
                {
                    direction = 3; // Left
                }
                PIDturn(90 * direction);
                wait(0.1, seconds);
                PIDmove(325);
                wait(0.1, seconds);
                dfs(grid, next_cell_x, next_cell_y, visit_Array);
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 50, "DEADEND %d:", cnt);
                Brain.Screen.printAt(10, 70, "[%d][%d]", dx, dy);
                wait(0.1, seconds);
                Brain.Screen.clearScreen();
                PIDturn(90 * direction);
                wait(0.1, seconds);
                PIDmove(-325);
                wait(0.1, seconds);
            }
            else
            {
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 50, "Checked from recusrion!!!");
                wait(2, seconds);
            }
        }
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 50, "DFS DONE!!!");
    }
}

// Copy the DFS grid into a second array with the axes reordered for easier indexing.
void Drivetrain::array_changer(int array1[][3], int array2[][3])
{
    for (int i = 0; i < grid_rows; i++)
    {
        for (int j = 0; j < grid_cols; j++)
        {
            array2[i][j] = array1[j][i];
        }
    }
}

// Find the location of a requested plant color in the map and flag whether it exists.
void Drivetrain::index_finder(int &x_pos, int &y_pos, int grid[][3], int colour_num, bool &check)
{
    x_pos = -1;
    y_pos = -1;
    check = false;

    for (int i = 0; i < grid_rows; i++)
    {
        for (int j = 0; j < grid_cols; j++)
        {
            if (grid[i][j] == colour_num)
            {
                x_pos = i;
                y_pos = j;
                check = true;
            }
        }
    }
}

// Build the path from the robot to the target square while avoiding already-checked cells.
void Drivetrain::mapping(int grid[][3], int &numcnt, bool &finalcheck, int &x_pos, int &y_pos, int &new_x, int &new_y, bool verify[][3], int &verifycnt, int movement[], int dead[], int wanted_x, int wanted_y)
{
    if (finalcheck)
    {
        int direction[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        int possible[4][2] = {{0}};
        int cnt = 0;
        x_pos = new_x;
        y_pos = new_y;
        for (int i = 0; i < 4; i++)
        {
            x_pos += direction[i][0];
            y_pos += direction[i][1];
            if (x_pos < 0 || y_pos < 0 || x_pos > (grid_rows - 1) || y_pos > (grid_cols - 1) || verify[x_pos][y_pos])
            {
            }
            else
            {
                if (grid[x_pos][y_pos] != 0)
                {
                    verify[x_pos][y_pos] = true;
                }
                else
                {
                    possible[cnt][0] = x_pos;
                    possible[cnt][1] = y_pos;
                    cnt++;
                }
                if (x_pos == wanted_x && y_pos == wanted_y)
                {
                    finalcheck = false;
                    if (x_pos - new_x == 0)
                    {
                        if (y_pos - new_y == 1)
                        {
                            movement[numcnt] = 1; // right
                        }
                        else
                        {
                            movement[numcnt] = 2; // left
                        }
                    }
                    else
                    {
                        if (x_pos - new_x == 1)
                        {
                            movement[numcnt] = 3; // down
                        }
                        else
                        {
                            movement[numcnt] = 4; // up
                        }
                    }
                }
            }
            x_pos = new_x;
            y_pos = new_y;
        }
        verify[x_pos][y_pos] = true;
        if (finalcheck)
        {
            for (int j = 0; j < cnt; j++)
            {
                if (finalcheck)
                {
                    verifycnt += 1;
                    new_x = possible[j][0];
                    new_y = possible[j][1];
                    verify[new_x][new_y] = true;
                    if (x_pos - new_x == 0)
                    {
                        if (y_pos - new_y == 1)
                        {
                            movement[numcnt] = 2; // left
                        }
                        else
                        {
                            movement[numcnt] = 1; // right
                        }
                    }
                    else
                    {
                        if (x_pos - new_x == 1)
                        {
                            movement[numcnt] = 4; // up
                        }
                        else
                        {
                            movement[numcnt] = 3; // down
                        }
                    }
                    numcnt++;
                    if (finalcheck)
                    {
                        mapping(grid, numcnt, finalcheck, x_pos, y_pos, new_x, new_y, verify, verifycnt, movement, dead, wanted_x, wanted_y);
                        if (finalcheck)
                        {
                            dead[numcnt - 1] = 1;
                            new_x = x_pos;
                            new_y = y_pos;
                        }
                    }
                }
            }
        }
    }
    else
    {
        numcnt = 0;
    }
}

// Follow the calculated route to the plant using the stored movement instructions.
void Drivetrain::GoToPos(int coming[], int finalcnt)
{
    for (int i = 0; i < finalcnt; i++)
    {

        if (coming[i] == 1)
        {
            PIDturn(90);
        }
        else if (coming[i] == 2)
        {
            PIDturn(270);
        }
        else if (coming[i] == 3)
        {
            PIDturn(180);
        }
        else
        {
            PIDturn(0);
        }
        if (i != (finalcnt - 1))
        {
            PIDmove(340);
        }
    }
}

// Reverse the route and return the robot to its starting position after watering.
void Drivetrain::comeHome(int coming[], int finalcnt)
{
    for (int i = 1; i < finalcnt; i++)
    {
        if (coming[i] == 1)
        {
            PIDturn(90);
        }
        else if (coming[i] == 2)
        {
            PIDturn(270);
        }
        else if (coming[i] == 3)
        {
            PIDturn(180);
        }
        else
        {
            PIDturn(0);
        }
        PIDmove(340);
    }
    wait(0.2, sec);
    PIDturn(0);
}