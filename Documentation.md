Documentation


Approach:

I implemented the classical centerline method.
Blue (left) and yellow (right) cones are paired into gates, the midpoint of each gate is a waypoint, 
and the waypoints are connected into a path with steps of 0.5 m.
When one side has fewer cones, I create virtual cones on that side by mirroring (offsetting) the real ones across the track
by the track width, so the same midpoint logic works for every case.
I added small comments in the code to explain each segment's general purpose



Code algorithm:

1. Sort cones: Cones are split by color and sorted by their straight line distance from the car, 
   ,so the nearest cone is first and blue/yellow cones are paired by index.

2. Track direction: I calculate it by getting a unit vector t from the first and last cone of a side:
   t = (last - first) / |last - first|. If one cone or one side only exists I take the line perpendicular to the line between them

3. Normal and track width: The normal pointing to the right (toward yellow) is n = (ty, -tx).
   The track width is then obtained by calculating average of the gate vectors multiplied by n (dot product) 
   If no full gate exists, width = 3 m.

4. Virtual cones: A missing yellow cone is blue + width * n, and a missing blue cone is yellow - width * n.

5. Waypoints: Each gate gives the midpoint between 2 different coloured cones. The path starts at the car position.

6. Interpolation. Between two waypoints, points are placed linearly, 
   we ensure that every step is  at or below 0.5 m.

7. Length control. If the path is under 5 m, it is extended. 
   If it is over 10 m, points are removed from the end. With no cones, the path goes straight along the car heading.




Design choice:

I did not use Delaunay triangulation or a Voronoi diagram.
Because at most we have 2 cones per side (and 3 on one side in Part 2),
so the input is too small for them to add anything, it will require much more computation power and complexity which would be redundant,
pairing cones by distance and taking midpoints gives the same centerline with far less code and less complexity. 
In a real competition setting, where the whole cone layout is visible, Triangulation and Voronoi are better suited. 
Delaunay edges between blue and yellow cones give the gates, and their midpoints (or the Voronoi edges between the two cone colors)
give the centerline. They can better handle sharp turns and can help the car return to the right track if it goes off course
however, in this simple example, it is not necessary.

Advantages:

- Simple to implement and computationally cheap
- The path stays centered between the cones when both sides are visible.
- It needs no training, and it runs instantly.
- Can work when we even have one cone only or one pair only

Disadvantages:

- The path consists of many straight line, so it has corners instead of a smooth curve.
- Direction comes from a straight line between two cones, which is only approximate on curves.
- Virtual cones depend on the track width, which is a guess when only one side is visible.

Limitations:

- Sorting by distance from the car can fail on turns, where a later cone is closer than an earlier one.
- With three cones on one side on a tight curve. Like scenario 15
  one normal is used for the whole side, so the virtual cones are misplaced and path may be inaccurate
- Two cones at the same position let the program assume zero direction, so that rule is skipped and the next one is used
- If the real track width differs from the estimated one, the path is incorrect
- If the car goes off course, it is hard for it to reroute to the center line again
- When the cones disagree about the track direction, the path can go wrong
  (scenario 15). The two blue cones suggest the track turns up and to the left,
  but the one real gate (blue and yellow) suggests it goes straight. The code
  uses the two blue cones for the direction, so the virtual yellow cone ends up
  in the wrong place.

Assumptions:

- The track width is 3 m when it cannot be measured. I searched for it on the internet :)
- With no cones visible, the best choice is to go straight depending on the car's yaw.

Part 2: three cones on one side

Approach:
No new code path was needed. When only one color is visible, the track direction
is computed from the first and last cone of that side, like I mentioned above. 
Every cone is mirrored by an offset by the assumed track width. 
The rest remains unchanged.

Why this approach:
I reused the Part 1 method, so there is only one algorithm to explain and test.
Fitting a curve through the three cones would follow bends better, but it is more
complicated and can fail in some cases and would take more computational power, so I kept it simple.


Added scenarios:
  21: three yellow cones in a straight line. Expected: straight path along the middle of the track.
  22: three yellow cones curving left. Expected: path bends left.
  23: three yellow cones curving right. Expected: path bends right.


Limitations:
- One direction (first cone to last cone) is used for the whole side. On a curve
  this is an average, so the path is slightly off near the ends.
  The sharper the curve, the larger the error.
- The track width is a guess, because no blue cone exists to measure it from.
- The extension (when the path is under 5 m) continues straight, so on a curve
  the extra part can leave the track.