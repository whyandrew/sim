/*
    Lander Control simulation.

    Updated by F. Estrada for CSC C85, Oct. 2013
    Mod by Per Parker, Sep. 2015

    Learning goals:

    - To explore the implementation of control software
      that is robust to malfunctions/failures.

    The exercise:

    - The program loads a terrain map from a .ppm file.
      the map shows a red platform which is the location
      a landing module should arrive at.
    - The control software has to navigate the lander
      to this location and deposit the lander on the
      ground considering:

      * Maximum vertical speed should be less than 5 m/s at touchdown
      * Maximum landing angle should be less than 10 degrees w.r.t vertical

    - Of course, touching any part of the terrain except
      for the landing platform will result in destruction
      of the lander

    This has been made into many videogames. The oldest one
    I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

    Your task:

    - These are the 'sensors' you have available to control
          the lander.

      Velocity_X();  - Gives you the lander's horizontal velocity
      Velocity_Y();  - Gives you the lander's vertical velocity
      Position_X();  - Gives you the lander's horizontal position (0 to 1024)
      Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();   - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

      SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

    - Variables accessible to your 'in flight' computer

      MT_OK     - Boolean, if 1 indicates the main thruster is working properly
      RT_OK     - Boolean, if 1 indicates the right thruster is working properly
      LT_OK     - Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X    - X position of the landing platform
          PLAY_Y        - Y position of the landing platform

    - Control of the lander is via the following functions
          (which are noisy!)

      Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
      Left_Thruster(double power);   - Sets left thruster power in [0 1]
      Right_Thruster(double power);  - Sets right thruster power in [0 1]
      Rotate(double angle);      - Rotates module 'angle' degrees clockwise
                       (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

                       Note that rotation takes time!


    - Important constants

      G_ACCEL = 8.87    - Gravitational acceleration on Venus
      MT_ACCEL = 35.0   - Max acceleration provided by the main thruster
      RT_ACCEL = 25.0   - Max acceleration provided by right thruster
      LT_ACCEL = 25.0   - Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

    - Functions you need to analyze and possibly change

      * The Lander_Control(); function, which determines where the lander should
        go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

    - You *can* add your own helper functions (e.g. write a robust thruster
      handler, or your own robust sensor functions - of course, these must
      use the noisy and possibly faulty ones!).

    - The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

             Initial lander position, orientation, and velocity are
                         randomized.

      * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

      * Failure modes: 0 - Nothing ever fails, life is simple
               1 - Controls can fail, sensors are always reliable
               2 - Both controls and sensors can fail (and do!)
               3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

        * Note - while running. Pressing 'q' on the keyboard terminates the 
            program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

    Have fun! try not to crash too many landers, they are expensive!

    Credits: Lander image and rocky texture provided by NASA
         Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Lander_Control.h"

// Constants
#define NUMSAMPLES 40
#define NUM_RECENT_STATES 10
#define MAX_VEL_DIFF 2.0  // Maximum span between velocity readings to be considered legit
#define MAX_POS_DIFF 50.0 // Maximum span between position readings to be considered legit
#define MAX_ANGLE_DIFF 20.0

// Sensor states variables
// True == working ok; False == something's wrong!
bool Vx_OK = true; // Velocity_X()
bool Vy_OK = true; // Velocity_Y()
bool PosX_OK = true;
bool PosY_OK = true;
bool Angle_OK = true;
bool Sonar_OK = true;

// Control states
bool isRotating = false;
double targetRotate = 0.0;

double emergency_tilt = 0.0;
unsigned long frame_count = 0;

struct State
{
    bool thr_L_OK; // Left thruster OK
    bool thr_M_OK; // Middle thruster OK
    bool thr_R_OK; // Right thruster OK
 
    bool vel_x_OK; // Velocity_X() OK
    bool vel_y_OK; // Velocity_Y() OK
    bool pos_x_OK; // Position_X() OK
    bool pos_y_OK; // Position_Y() OK
    bool angle_OK;
    bool sonar_OK;

    double pow_L; // Left thruster power
    double pow_M; // Middle thruster power
    double pow_R; // Right thruster power

    double rot_angle; // Angle of last rotation call
    unsigned long rot_frame; // Frame at which Rotate() was last called

    double vel_x;
    double vel_y;
    double pos_x;
    double pos_y;
    double angle;
    double sonar[36];
    double range; // RangeDist()
};

struct State *recent_states[NUM_RECENT_STATES];

// Functions to update the current State struct

void Update_Velocity_X()
{
    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state
    double sum = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double curr_reading;

    for (int i = 0; i < NUMSAMPLES; i++)
    {
        curr_reading = Velocity_X();
        
        if (i==0)
        {
            minimum = maximum = curr_reading;
        }
        else
        {
            maximum = fmax(maximum, curr_reading);
            minimum = fmin(minimum, curr_reading);
        }
        sum += curr_reading;
    }
    if (maximum - minimum < MAX_VEL_DIFF) // Biggest span between values seems fine
    {
        recent_states[state_idx]->vel_x_OK = true;
        recent_states[state_idx]->vel_x = (sum / NUMSAMPLES);
        Vx_OK = true;
    }
    else // Biggest span between values too high, reading suspect
    {
        recent_states[state_idx]->vel_x_OK = false;
        recent_states[state_idx]->vel_x = 0;// TODO: replace w/ calculation from past values
        Vx_OK = false;
    }
    // if (!Vx_OK)
    //     printf("Update_Velocity_X:: min %f max %f diff %f\nseems legit? %s\n\n",
    //        minimum, maximum, maximum - minimum,
    //        recent_states[state_idx]->vel_x_OK ? "true" : "false");
}

void Update_Velocity_Y()
{
    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state
    double sum = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double curr_reading;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        curr_reading = Velocity_Y();
        if (i==0)
        {
            minimum = maximum = curr_reading;
        }
        else
        {
            maximum = fmax(maximum, curr_reading);
            minimum = fmin(minimum, curr_reading);
        }
        sum += curr_reading;
    }
    if (maximum - minimum < MAX_VEL_DIFF) // Biggest span between values seems fine
    {
        recent_states[state_idx]->vel_y_OK = true;
        recent_states[state_idx]->vel_y = (sum / NUMSAMPLES);
        Vy_OK = true;
    }
    else // Biggest span between values too high, reading suspect
    {
        recent_states[state_idx]->vel_y_OK = false;
        recent_states[state_idx]->vel_y = 0;// TODO: replace w/ calculation from past values
        Vy_OK = false;
    }
    // if (!Vy_OK)
    //     printf("Update_Velocity_Y: min %f max %f diff %f\nseems legit? %s\n\n",
    //        minimum, maximum, maximum - minimum,
    //        recent_states[state_idx]->vel_y_OK ? "true" : "false");
}

void Update_Position_X()
{
    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state
    double sum = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double curr_reading;
     for (int i = 0; i < NUMSAMPLES; i++)
    {
        curr_reading = Position_X();
        
        if (i==0)
        {
            minimum = maximum = curr_reading;
        }
        else
        {
            maximum = fmax(maximum, curr_reading);
            minimum = fmin(minimum, curr_reading);
        }
        sum += curr_reading;
    }

    if (maximum - minimum < MAX_POS_DIFF) // Biggest span between values seems fine
    {
        recent_states[state_idx]->pos_x_OK = true;
        recent_states[state_idx]->pos_x = (sum / NUMSAMPLES);
        PosX_OK = true;
    }
    else // Biggest span between values too high, reading suspect
    {
        recent_states[state_idx]->pos_x_OK = false;
        recent_states[state_idx]->pos_x = 0;// TODO: replace w/ calculation from past values
        PosX_OK = false;
    }
    // if (!PosX_OK)
    //     printf("Update_Position_X: min %f max %f diff %f\nseems legit? %s\n\n",
    //        minimum, maximum, maximum - minimum,
    //        recent_states[state_idx]->pos_x_OK ? "true" : "false");
}

void Update_Position_Y()
{
    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state
    double sum = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double curr_reading;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        curr_reading = Position_Y();
        if (i==0)
        {
            minimum = maximum = curr_reading;
        }
        else
        {
            maximum = fmax(maximum, curr_reading);
            minimum = fmin(minimum, curr_reading);
        }
        sum += curr_reading;
    }
    if (maximum - minimum < MAX_POS_DIFF) // Biggest span between values seems fine
    {
        recent_states[state_idx]->pos_y_OK = true;
        recent_states[state_idx]->pos_y = (sum / NUMSAMPLES);
        PosY_OK = true;
    }
    else // Biggest span between values too high, reading suspect
    {
        recent_states[state_idx]->pos_y_OK = false;
        recent_states[state_idx]->pos_y = 0;// TODO: replace w/ calculation from past values
        PosY_OK = false;
    }
    // if (!PosY_OK)
    //     printf("Update_Position_Y: min %f max %f diff %f\nseems legit? %s\n\n",
    //        minimum, maximum, maximum - minimum,
    //        recent_states[state_idx]->pos_y_OK ? "true" : "false");
}

// Robust APIs for all sensors
double Robust_Velocity_X()
{
    double sum = 0.0;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        sum += Velocity_X();
    }
    return (sum / NUMSAMPLES);
}

double Robust_Velocity_Y()
{
    double sum = 0.0;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        sum += Velocity_Y();
    }
    return (sum / NUMSAMPLES);
}

double Robust_Position_X()
{
    double sum = 0.0;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        sum += Position_X();
    }
    return (sum / NUMSAMPLES);
}

double Robust_Position_Y()
{
    double sum = 0.0;
    for (int i = 0; i < NUMSAMPLES; i++)
    {
        sum += Position_Y();
    }
    return (sum / NUMSAMPLES);
}

// Angle from 0 to 360 degrees w.r.t vertical up
double Robust_Sonar(double angle)
{
    // TODO
    // Now only takes single reading since description says Sonar reading takes time to update (how often tho?)
    // Use linear interpolation, should be mostly ok except in sharp pointy terrain.
    while (angle >= 360.0)
    {
        angle -= 360.0;
    }
    while (angle < 0.0)
    {
        angle +=360.0;
    }

    // super special case is angle = 0.0000; 360.0 will be changed to 0.000 above
    if (0.000 == angle)
    {
        return SONAR_DIST[0];
    }

    double angleSeg = angle/10.0; // 0.0~ to 35.9~
    int topIndex = 1 + (int)(angleSeg); // 1 to 36
    int botIndex = (int)(angleSeg); // 0 to 35
    double decimals = angleSeg - botIndex;
    double ret;

    if (0.0 != decimals)
    {
        // Intrapolate 2 indexes of SONAR_DIST
        return SONAR_DIST[botIndex] + ((SONAR_DIST[topIndex] - SONAR_DIST[botIndex]) * decimals);
    }
    else
    {
        // Cases when angle is exact multiples of 10.0 degrees
        return SONAR_DIST[botIndex];
    }

    return ret;
}

double Robust_Angle()
{
    // Corner case when angle is near 0 (360) degrees:
    // It appears Lander_Control API will always return Angle() either 
    // using 0-range (can be negative angle), 360-range (can be more than 360)
    // on a single call into Lander_Control.
    // This makes averaging easy since we won't be averaging, say, 359 and 1 degrees
    double ret = 0.0;
    int counter;

    counter = (Angle_OK)? NUMSAMPLES: NUMSAMPLES * 100;

    for (int i = 0; i < counter; i++)
    {
        ret += Angle();
    }

    ret = ret / counter;

    while (ret >= 360.0)
    {
        ret -= 360.0;
    }
    while (ret < 0.0)
    {
        ret += 360.0;
    }
    return ret;
}

double Robust_RangeDist()
{
    //RangeDist() never fails, and it appears to be always accurate... or is it?

    // RangeDist reads from the direction of the main thruster
    if (Angle_OK)
    {
        return RangeDist();
    }
    else // if Angle sensor not working, don't really know where we're point at...
    {
        //TODO .... what to do?!
        return RangeDist();
    }
}

void Update_Angle()
{
    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state
    //Check for bad Angle sensor
    if (Angle_OK)
    {
        double min = 999999.0;
        double max = -9999999.0;
        double reading;
        for (int i = 0; i < NUMSAMPLES; i++)
        {
            reading = Angle();
            if (reading > max)
            {
                max = reading; 
            }
            if (reading < min)
            {
                min = reading;
            }
        }

        if ((max - min) > MAX_ANGLE_DIFF)
        {
            Angle_OK = false;
        }
    }

    recent_states[state_idx]->angle_OK = Angle_OK;
    // Angle sensor still works when "faulty"... just alot more noise.
    recent_states[state_idx]->angle = Robust_Angle();
}

void Log_sensors()
{
    /* Update state variables */

    int state_idx = frame_count % NUM_RECENT_STATES; // index of current state

    Update_Velocity_X();
    Update_Velocity_Y();
    //printf("\n");
    Update_Position_X();
    Update_Position_Y();
    Update_Angle();
    //printf("\n\n******************************************************\n");

    recent_states[state_idx]->thr_L_OK = LT_OK; 
    recent_states[state_idx]->thr_M_OK = MT_OK;
    recent_states[state_idx]->thr_R_OK = RT_OK;
}

double Corrected_Angle(void)
{
    /* Return the ship's angle to vertical, corrected so that the
    orientation considered vertical is one chosen so as to
    preserve manouverability in spite of any thruster failures.
    
   Applicable only when angle sensor is functioning. */

    double corrected = Robust_Angle();
    double x_error = Robust_Position_X() - PLAT_X;
    double k_x = 1.0;   // K value for proportional component in rocket aiming
    double k_vx = 15.0; // K value for derivative (velocity) component
    
    double tilt;
    double height_from_platform = PLAT_Y - Robust_Position_Y();
    double min_height = 30.0; // Minimum height at which angle adjustments
                            // apply. Below this, the lander is so close
                            // to the landing pad that it should reorient
                            // normally to avoid crashing.


    if (emergency_tilt == 0)
        tilt = 360.0 * atan(k_x * x_error + k_vx * Robust_Velocity_X()) - 180;
    else
        tilt = emergency_tilt;
    //printf("tilt: %f emergency_tilt: %f\n", tilt, emergency_tilt);


    if (LT_OK && MT_OK && !RT_OK && (height_from_platform > min_height))
    {   // Two adjacent thrusters (left and middle) are working, so
        // we want to orient so that they are -45 and +45 from 180 (down)
        corrected += 45.0;
    }
    else if (!LT_OK && MT_OK && RT_OK && (height_from_platform > min_height))
    {   // Two adjacent thrusters (middle and right) are working, so
        // we want to orient so that they are -45 and +45 from 180 (down)
        corrected -= 45.0;  
    }
    else if (LT_OK && !MT_OK && !RT_OK && (height_from_platform > min_height))
    {   // Only the left thruster is working, so we want to orient so that
        // it points down.
        corrected += 90.0;
        corrected += tilt; 
    }
    else if (!MT_OK && RT_OK && (height_from_platform > min_height))
    {   // The middle thruster is out and the right thruster still works,
        // so we orient so that the right thruster points down. The left
        // thruster may or may not work, but without a middle thruster
        // we need either the right or the left thruster pointing down,
        // so right is chosen arbitrarily.
        corrected -= 90.0;
        corrected += tilt;
    }
    else if (!LT_OK && MT_OK && !RT_OK && (height_from_platform > min_height))
    {   // The middle thruster is the only one working, so we orient so that
        // the middle thruster points down. 
        corrected += tilt; 
    }
    else // Either all thrusters work, none work (!), or
         // height_from_platform <= min_height. In each of these
         // cases, normal orientation is the best choice. 
    {
        
    }
    while (corrected >= 360)
        corrected -= 360.0;
    while (corrected < 0)
        corrected += 360.0;
    return corrected;
}

void Single_Thruster(double power)
{
    /* If the middle and right thrusters are out and the left one works,
       activate the left thruster using power value power.

       If the middle thruster is out and the right thruster works,
       use the right thruster.

       If the middle thruster works and both the others are out, use
       the middle thruster.
       
       If the middle thruster and at least one other thruster work,
       print an error message to stderror explaining that Single_Thruster
       is not intended for use in this situation.
    */
    double side_thruster_power = fmin(1.0, (35.0/25.0) * power);

    if (LT_OK && !MT_OK && !RT_OK)
    {
        Left_Thruster(side_thruster_power);
    }
    else if (!MT_OK && RT_OK)
    {
        Right_Thruster(side_thruster_power);
        Left_Thruster(0);
    }
    else if (!LT_OK && MT_OK && !RT_OK)
    {
        Main_Thruster(power);
    }
    else
    {
        fprintf(stderr, "Error: Single_Thruster() is not intended to be used\n"
                        "when two adjacent thrusters are working.\n");
    }
}

void Lander_Control(void)
{
     /*
       This is the main control function for the lander. It attempts
       to bring the ship to the location of the landing platform
       keeping landing parameters within the acceptable limits.

       How it works:

       - First, if the lander is rotated away from zero-degree angle,
         rotate lander back onto zero degrees.
       - Determine the horizontal distance between the lander and
         the platform, fire horizontal thrusters appropriately
         to change the horizontal velocity so as to decrease this
         distance
       - Determine the vertical distance to landing platform, and
         allow the lander to descend while keeping the vertical
         speed within acceptable bounds. Make sure that the lander
         will not hit the ground before it is over the platform!

       As noted above, this function assumes everything is working
       fine.
    */

    //printf(".........................................................................\n");
    // for (int i = 0; i<10; i++)
    // {
    //     printf("X:%f; Y:%f; Vx:%f; Vy:%f; Angle:%f; RangeDist:%f\n",
    //         Robust_Position_X(), Robust_Position_Y(), Robust_Velocity_X(), Robust_Velocity_Y(),
    //         Robust_Angle(), Robust_RangeDist() );
    // }

    // for (int i = 10; i < 27; i++)
    // {
    //     printf("%3.1f, ",            SONAR_DIST[i]);
    // }
    // printf("\n");

    for (int i = 0; i<10; i++)
    {
        printf("Angle:%f; Robust_Angle:%f\n",
            Angle(), Robust_Angle() );
    }
    printf(".........\n");

    double VXlim;
    double VYlim;
    
    if (frame_count == 0)
    {
        for (int i=0; i<NUM_RECENT_STATES; i++)
        {
            recent_states[i] = (struct State *)calloc(sizeof(struct State), 1);
        }
    }
    Log_sensors(); 
    frame_count++;

    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.
    if (fabs(Robust_Position_X()-PLAT_X)>200) 
        VXlim=25;
    else if (fabs(Robust_Position_X()-PLAT_X)>100) 
        VXlim=15;
    else 
        VXlim=5;

    if (PLAT_Y-Robust_Position_Y()>200) 
        VYlim=-20;
    else if (PLAT_Y-Robust_Position_Y()>100) 
        VYlim=-10;  // These are negative because they
    else 
        VYlim=-4;                     // limit descent velocity

    // Ensure we will be OVER the platform when we land
    if (fabs(PLAT_X-Robust_Position_X()) / fabs(Robust_Velocity_X()) > 
        1.25 * fabs(PLAT_Y-Robust_Position_Y()) / fabs(Robust_Velocity_Y())
        && (MT_OK && (RT_OK || LT_OK ))) // The VYlim = 0 was causing a vertical stalling
    {                                    // issue in single thruster mode
        VYlim=0;
    }

    // IMPORTANT NOTE: The code below assumes all components working
    // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
    // fail. More likely, you will need a set of case-based code
    // chunks, each of which works under particular failure conditions.

    // Check for rotation away from zero degrees - Rotate first,
    // use thrusters only when not rotating to avoid adding
    // velocity components along the rotation directions
    // Note that only the latest Rotate() command has any
    // effect, i.e. the rotation angle does not accumulate
    // for successive calls.

    if (Corrected_Angle()>(1)&&Corrected_Angle()<359)
    {
        if (Corrected_Angle()>=180) 
            Rotate(360-Corrected_Angle());
        else 
            Rotate(-Corrected_Angle());
        return;
    }

    // Module is oriented properly, check for horizontal position
    // and set thrusters appropriately.
    if (Robust_Position_X()>PLAT_X)
    {   
        
        if (MT_OK && (RT_OK || LT_OK )) // If the middle thruster and at least one of the others
        {                               // are functioning, use the original landing code
            // Lander is to the LEFT of the landing platform, use Right thrusters to move
            // lander to the left.
            Left_Thruster(0);   // Make sure we're not fighting ourselves here!
            if (Robust_Velocity_X()>(-VXlim)) 
                Right_Thruster((VXlim+fmin(0,Robust_Velocity_X()))/VXlim);
            else
            {
                // Exceeded velocity limit, brake
                Right_Thruster(0);
                Left_Thruster(fabs(VXlim-Robust_Velocity_X()));
            }
        }
        else
        {
            // Single thruster mode: nothing to do, Corrected_Angle has aimed the thruster
            // as needed for the desired horizontal direction
        }
    }
    else
    {
        if (MT_OK && (RT_OK || LT_OK )) // If the middle thruster and at least one of the others
        {
            // Lander is to the RIGHT of the landing platform, opposite from above
            Right_Thruster(0);
            if (Robust_Velocity_X()<VXlim) 
                Left_Thruster((VXlim-fmax(0,Robust_Velocity_X()))/VXlim);
            else
            {
                Left_Thruster(0);
                Right_Thruster(fabs(VXlim-Robust_Velocity_X()));
            }
        }
        else
        {
            // Single thruster mode
            //Single_Thruster(1.0);
        }
    }

    // Vertical adjustments. Basically, keep the module below the limit for
    // vertical velocity and allow for continuous descent. We trust
    // Safety_Override() to save us from crashing with the ground.
    if (MT_OK && (RT_OK || LT_OK ))
    {
        if (Robust_Velocity_Y()<VYlim) 
            Main_Thruster(1.0);
        else 
            Main_Thruster(0);
    }
    else
    {
        if (Robust_Velocity_Y()<VYlim) 
            Single_Thruster(1.0);
        else 
            Single_Thruster(0);
    }
}

void Safety_Override(void)
{
    /*
    This function is intended to keep the lander from
    crashing. It checks the sonar distance array,
    if the distance to nearby solid surfaces and
    uses thrusters to maintain a safe distance from
    the ground unless the ground happens to be the
    landing platform.

    Additionally, it enforces a maximum speed limit
    which when breached triggers an emergency brake
    operation.
    */

/**************************************************
    TO DO: Modify this function so that it can do its
            work even if components or sensors
            fail
**************************************************/

/**************************************************
    How this works:
    Check the sonar readings, for each sonar
    reading that is below a minimum safety threshold
    AND in the general direction of motion AND
    not corresponding to the landing platform,
    carry out speed corrections using the thrusters
**************************************************/

    //printf("Safety_Override\n");

    double DistLimit;
    double Vmag;
    double dmin;

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag=Robust_Velocity_X()*Robust_Velocity_X();
    Vmag+=Robust_Velocity_Y()*Robust_Velocity_Y();

    DistLimit=fmax(75,Vmag);

    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
    if (fabs(PLAT_X-Robust_Position_X()) < 150 && fabs(PLAT_Y-Robust_Position_Y()) < 150) 
        return;

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.
    dmin=1000000;
    if (Robust_Velocity_X()>0)
    {
        for (int i=5;i<14;i++)
            if (SONAR_DIST[i]>-1&&SONAR_DIST[i] < dmin) 
                dmin = SONAR_DIST[i];
    }
    else
    {
        for (int i=22;i<32;i++)
            if (SONAR_DIST[i]>-1&&SONAR_DIST[i] < dmin) 
                dmin = SONAR_DIST[i];
    }
    // Determine whether we're too close for comfort. There is a reason
    // to have this distance limit modulated by horizontal speed...
    // what is it?
    if (dmin < DistLimit*fmax(.25, fmin(fabs(Robust_Velocity_X())/5.0, 1)))
    { // Too close to a surface in the horizontal direction
        //printf("Too close to a surface in the horizontal direction: %f\n", dmin);
        if (Corrected_Angle() > 1 && Corrected_Angle() < 359)
        {
            if (Corrected_Angle()>=180) 
                Rotate(360-Corrected_Angle());
            else 
                Rotate(-Corrected_Angle());
            return;
        }

        if (Robust_Velocity_X()>0)
        {
            if  (MT_OK && (RT_OK || LT_OK ))
            {
                Right_Thruster(1.0);
                Left_Thruster(0.0);
            }
            else
            {
                emergency_tilt = 75.0;
                Single_Thruster(0.5);
            }
        }
        else
        {
            if  (MT_OK && (RT_OK || LT_OK ))
            {
                Left_Thruster(1.0);
                Right_Thruster(0.0);
            }
            else
            {
                emergency_tilt = -75.0;
                Single_Thruster(0.5);
            }
        }
    }
    else
    {
        emergency_tilt = 0.0;
    }

    // Vertical direction
    dmin=1000000;
    if (Robust_Velocity_Y()>5)      // Mind this! there is a reason for it...
    {
        for (int i=0; i<5; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) 
                dmin = SONAR_DIST[i];

        for (int i=32; i<36; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) 
                dmin = SONAR_DIST[i];
    }
    else
    {
        for (int i=14; i<22; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) 
                dmin = SONAR_DIST[i];
    }
    if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
    {
        if (Corrected_Angle() > 1 || Corrected_Angle() > 359)
        {
            if (Corrected_Angle() >= 180) 
                Rotate(360-Corrected_Angle());
            else 
                Rotate(-Corrected_Angle());
            return;
        }
        if (Robust_Velocity_Y()>2.0){
            Main_Thruster(0.0);
        }
        else
        {
            if (MT_OK && (RT_OK || LT_OK ))
            {
                Main_Thruster(1.0);
            }
            else
            {
                Single_Thruster(1.0);
            }
        }
    }
}
