
Sensor failures

4 - Horizontal velocity sensor: *(OK NOW)*
  Usually seems to land fine.

5 - Vertical velocity sensor:
  Comes close to landing but won't lower itself down the last little bit.

6 - Horizontal position sensor: *(OK NOW)*
  Usually seems to land fine.

7 - Vertical position sensor: **(OK NOW)**
  Need more testing in different failure combo...

8 - Angle sensor:
    Usually seems to land fine.

9 - Sonar:
With no correction, the absence of sonar only really seems to cause problems on the hard map. The copy of Lander.cpp in /Test_only/ now seems to scan properly. It calculates the minimum distance for each quadrant directly as used by safety_override (dmin), updating the global variables min_dist_U, min_dist_L, min_dist_D, and min_dist_R (up, left, down, right). Still to do: better determination of when to scan: prevent scanning when close to landing, when sonar is known to be working, etc. Also address failure to descend that happens sometimes (scanning too frequently?) Then incorporate into main file.

Lander_Control easy.ppm 3 2 4 6 - without left thruster & no pos_x vel_x, always crash just to right. Use single thruster mode instead?

Lander_Control easy.ppm 3 3 4 5 - without right thruster & no pos_x vel_x, always crash just to left. Use single thruster mode instead?
