## Project: Search and Sample Return
---

[warped]: writeup/warped.jpg 
[obstacle]: writeup/obstacle.jpg 
[navigable]: writeup/navigable.jpg 
[sample]: writeup/sample.jpg
[sample_threshed]: writeup/sample_threshed.jpg

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Obstacle Detection
An RGB threshold was applied to each image after the perspective transform in order to find obstacles. Since the obstacles 
were all lighter than the drivable surface, any pixel with RGB values all over 160 was assigned as a drivable area. Any
pixel that was not drivable was considered an obstacle. Any values behind the rover's field of view was considered to be
 neither an obstacle nor a navigable area. For example, look at the following image after a perspective transform. 
 
 *Warped Image*
 
 ![Warped][warped]
 
 
 Notice the black areas in the bottom left and right. These pixels can't be seen by the camera, so they appear black after 
 applying the perspective transform.
 
 *Navigable Area*
 
 ![Navigable]
 
 This is the navigable area. These pixels are identified by having RGB values all above 160.
 
 *Obstacles*
 
 ![Obstacle]
 
 The obstacle area is the inverse of the navigable area, except for the bottom left and right areas. After taking the 
 inverse of the navigable area, any pixel that is (0,0,0) in the warped image is set to 0 here.
 
 

#### 2. Sample Detection
Sample detection is also based on the color profile of the image, but it works using HSV instead of RGB. In the HSV 
color space, the base color is controlled by only one value (the Hue), and the other two control details about its appearance.
Any pixel with HSV values within a specified range is considered a sample. Setting a small Hue range allows the computer
 to reject anything that's not yellow. My HSV ranges were as follows:
 
| Value | Min | Max |
|-------|-----|-----|
| Hue   | 15  | 30  |
|Saturation| 180 | 255|
|Value| 130 | 180 |

*Rock Sample*

![Rock sample][sample]

*Rock Sample after HSV Filtering*

![After Filtering][sample_threshed]

#### 3. Processing Images
Inside the `process_image()` function, I added all of the necessary steps to convert a picture from the rover into usable
map data. The steps were as follows:
- Perform perspective transform
- Identify obstacles and navigable areas using RGB filtering
- Identify samples using HSV filtering
- Transform obstacle, navigable, and sample maps into rover coordinates, e.g. with the rover at 0,0 looking to the right.
- Get rover position and yaw
- Rotate and translate data from rover frame to world frame
- Update world map with data

These steps successfully took the rover's field of vision and used it to add obstacle and sample data to the world map.

### Autonomous Navigation and Mapping

#### 1. Perception Step
For the perception step, I first followed all of the steps from processing images in the notebook. After that, I made 
two large changes: trimming the input image and using nav data to remove obstacles.

I noticed that the perspective transform gets less accurate farther away from the robot because it tries to stretch a 
very small part of the robot's field of vision into a version large part of the map. In particular, it would 
assign navigable areas as obstacles. Since this was messing up my navigation (see part 2), I decided to ignore all pixels
within 80 pixels of the top, right, or left. This made mapping less efficient, but it lead to a huge increase in map
fidelity.

Second, if an are aon the map was navigable in a particular image, I also reduced its value in the obstacle layer of the map.
Areas obscured by obstacles are incorrectly counted as obstacles, so this was my way to correct the map after seeing an 
area from the other side. 

#### 2. Decision Step

Unlike the perception step, my decision step is very different from the example code. By pure coincidence, I've already made
robot that can identify and pick up bright yellow balls, so I decided to focus on navigation, which my previous robot was
completely lacking. I did a lot of experimentation to find a good navigation system, and even it's certainly not perfect,
it was a good experience.

Instead of just turning into open space, I choose a destination point a at each iteration. The destination point is a point
that the rover has never been near, and it's chosen by prioritizing points that are nearby, in front of the robot, and 
away from walls.

Once the destination is chosen, I use D* Lite to find a path to it. At first I used A*, but D* Lite can re-use previous
calculations to save time when navigating far away. This gave a significant performance improvement, but I still saw 
performance problems (more on that later). The edge costs for the D* Lite algorithm were calculated by applying a gaussian 
filter to the obstacle layer of the map. This makes navigable points near walls have a higher cost, and allows D* Lite to
create paths that stay far away from walls.

Getting A* and D* Lite to work with camera data was very difficult for me to do, but I'm glad I spent a lot of time on it.

Once a path to the destination is chosen, the robot adjusts its steering angle to face near it, and the robot stops to 
rotate in place if its required steering angle is too high.


#### 3. Results  

*All my results are from running the simulator at 1600x1024 at Good Quality. FPS Varies, but I'll explain that later.*

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

Overall, the rover does a great job of exploring the entire map and avoiding obstacles. My D* Lite navigation system 
avoids obstacles and moves to all of the corners of the map systematically. That's good news, since almost all of my 
development time went into getting A* and D* Lite to work.

#### 4. Future Improvements

##### Navigation Performance
First of all, the biggest improvement would be better navigation performance. D* Lite causes lag of several seconds when 
it first calculates a path to a distant location. This is obvious when the rover runs into a dead end and has to turn 
around. Repeat calculations at long distances are much faster, but they're still too slow.

When the frame rate lags, it takes the robot much longer to update its obstacle map, so the rover will sometimes make 
paths that go through unseen obstacles. After a while, the map will update sufficiently and D* Lite will find the correct
path. Performance could be improved by only using a subset of map points for navigation, or by using a completely different
algorithm like RRT.

Also, the rover doesn't handle well at lower frames rates.

I'd love to come back to this after doing the later projects and see how much I can improve it.

##### Path
D* Lite generates a path based on moving between adjacent squares, which can be difficult
for a wheeled robot to follow. A better solution might be to use an implementation of RRT optimized for wheeled robots. 
That would give a path designed for a rover constrained to steering angles between -15 and 15 degrees.

##### Obstacle Detection
In narrow areas, the rover tends to think that distant shadows are obstacles. This could be fixed by updating the RGB 
threshold or further restricting the usable area of the camera's vision.

