
Sensor failures

4 - Horizontal velocity sensor: *(OK NOW)*
  Usually seems to land fine.

5 - Vertical velocity sensor:
  Comes close to landing but won't lower itself down the last little bit.

6 - Horizontal position sensor: *(OK NOW)*
  Usually seems to land fine.

7 - Vertical position sensor:
  Comes close to landing but won't lower itself down the last little bit.

8 - Angle sensor:
    Usually seems to land fine.

9 - Sonar:
  Can usually land on easy map and sometimes on hard even without fixing anything. The copy of Lander.cpp in /Test_only/ stops the lander at regular intervals and does the rotation w/ little y displacement, but there are some bugs to iron out (gets stuck partway through sometimes.) Still need to log values from rangedist, maybe just find the minimum values of certain ranges directly (and store to globals) instead of emulating the whole sonar array.
  
  e.g. min distance reading over 45-135 degrees -> min_dist_right

  min distance reading over 135-225 degrees -> min_dist_down
  
  min distance reading over 225-315 degrees -> min_dist_left
  
  min distance reading over 315-45 degrees -> min_dist_up
  
  Then maybe adjust these for displacement since scan.

Lander_Control easy.ppm 3 2 4 6 - without left thruster & no pos_x vel_x, always crash just to right. Use single thruster mode instead?

Lander_Control easy.ppm 3 3 4 5 - without right thruster & no pos_x vel_x, always crash just to left. Use single thruster mode instead?
