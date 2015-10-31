Tests
------

easy.ppm
(2 sensors)
  5	7	elvis
  2	5	hovers
  3	5	hovers

hard.ppm
(2 sensors)
4	9	crash
5	7	crash
5	9	crash
6	8	crash
6	9	crash
7	8	crash
7	9	crash
8	9	crash

(1 Thruster & 1 Sensor)
2	5	hovers
1	5	elvis
3	5	elvis

Sensor failures

4 - Horizontal velocity sensor: *(OK NOW)*
  Usually seems to land fine.

5 - Vertical velocity sensor: **(Workaround for case when Pos_Y is ok)**
  Andrew: Use Pos_Y to estimate velocity and average with calculated velocity from acceleration. Still can't find cause of issue, but I find the calculated velocity from acceleration is quite far off when lander approaches pad. Can possible be due to accumulated error?! ** Patrick: test more cases of bad vertical velocity sensor ** I only tried a few.

6 - Horizontal position sensor: *(OK NOW)*
  Usually seems to land fine.

7 - Vertical position sensor: **(OK NOW)**
  Need more testing in different failure combo...

8 - Angle sensor:
    Usually seems to land fine.

9 - Sonar:
With no correction, the absence of sonar only really seems to cause problems on the hard map. The copy of Lander.cpp in /Test_only/ now seems to scan properly. It calculates the minimum distance for each quadrant directly as used by safety_override (dmin), updating the global variables min_dist_U, min_dist_L, min_dist_D, and min_dist_R (up, left, down, right). Still to do: better determination of when to scan: prevent scanning when close to landing, when sonar is known to be working, etc. Also address failure to descend that happens sometimes (scanning too frequently?) Then incorporate into main file.


*Not important*:

Lander_Control easy.ppm 3 2 4 6 - without left thruster & no pos_x vel_x, always crash just to right. Use single thruster mode instead?

Lander_Control easy.ppm 3 3 4 5 - without right thruster & no pos_x vel_x, always crash just to left. Use single thruster mode instead?
