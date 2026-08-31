#pragma once
#include "iq2_cpp.h"
using namespace vex;

class Drivetrain
{
private:
    motor left_;
    motor right_;
    inertial BrainInertial;
    brain Brain;
    distance DistanceSensor;
    optical ColourSensor;
    touchled TouchSensor;
    timer timeout;
    int grid_rows;
    int grid_cols;

public:
    Drivetrain(char left_Port, char right_Port, char distanceSensor_port, char colourSensor_port, char touchSensor_port);
    ~Drivetrain();
    void setGrid(int x, int y);
    void stop();
    void PIDmove(float distance, float kp = 0.4, float ki = 0.01, float kd = 0.02);
    void PIDturn(float angle, float kp = 0.4, float ki = 0.01, float kd = 0.02);
    bool checkForPlant();
    int moveToPlant();
    int colourtotime(int colourValue);
    void pourwater(int time);
    void touchandgo();
    void dfs(int grid[][3], int &current_x_pos, int &current_y_pos, bool visit_Array[][3]);
    void index_finder(int& x_pos, int& y_pos, int grid[][3], int colour_num, bool& check);
    void mapping(int grid[][3], int &numcnt, bool &finalcheck, int &x_pos, int &y_pos, int &new_x, int &new_y, bool verify[][3], int &verifycnt, int movement[], int dead[], int wanted_x, int wanted_y);
    void GoToPos(int path[], int finalcnt);
    void comeHome(int path[], int finalcnt);
    void displayHue();
    void WateringPosition(float &distance_initial);
    void array_changer(int array1[][3], int array2[][3]);
};