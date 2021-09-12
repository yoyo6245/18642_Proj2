Description of files in the ece642rtle project directory.
Written by Milda Zizyte.
THIS FILE VIEWED BEST USING FIXED-WIDTH FONT

**** File structure: ****

images/		PNG images of the turtles (provided by the original turtlesim code from ROS.org)
include/	Header files for the maze, turtle frame, and turtle
monitors/	Runtime monitors
msg/		Definitions of ROS messages
src/		Project backend for turtle and maze
srv/		Definitions of ROS services
student/	Student code (including ros_turtle_interface)
ta/ 		Documentation for TAs
test/		Unit testing example

**** Nodes: ****

ece642rtle_node		The ROS backend node that does all the rendering and provides location/maze information. Students do not touch this.
			
ece642rtle_student	The student node, that returns a new position (and possibly displays the number of visits) for the turtle and queries for bumped(...), at_end(...) according to student implementation

**** Messages and Services: ****

(See also the "Communication between nodes" section below)

PoseOrntBundle.msg	Fields for timestamp, x and y coords, and orientation, used to communicate where the turtle is and what it's facing
timeInt8.msg		Int8 message with a timestamp wrapper (used for visit counts)
*Echo.msg		echos the corresponding service call and return. Used for runtime monitoring. (nodes cannot "subscribe" to listen in on services, which is why these are necessary)

RTIatend.srv		AtEnd call (takes an x and y, returns whether this is the end square. ece642rtle_node owns this service, using the Maze class to provide an answer.
RTIbump.srv		Bumped call (takes x1, x2, y1, y2, returns whether this line segment is a wall in the maze0. ece642rtle_node owns this service, using the Maze class to provide an answer.

**** Description of files in src/: ****

ece642rtle.cpp	      ROS code to initialize a TurtleFrame (bare bones starting of ROS application, barely changed from the original turtlesim code from ROS.org)

maze.cpp	      Reads in a Mazefile (see Mazefile spec below), which is taken using a ROS param (default is "m1.maze"). Reads the file line by line into a big array and then renders it. Returns methods to see if a line segment is a wall in the maze. Also stores and renders the start and end cells.

turtle.cpp	      Where the magic happens for the backend. Subscribes and advertises the necessary ROS communications, dealing with moves by rotating the turtle, updating the global knowledge of the turtle location, and painting the turtle. Also where messages for ece642rtle_node are sent and received.

turtle_frame.cpp      Renders the maze and spawns the turtle based on information in the Maze and Turtle classes. Converts from 0...12 maze cell coordinates to pixel coordinates.

**** Communication between nodes: ****

This is implemented in turtle.cpp (for ece642rtle_node side) and ros_turtle_interface.cpp (for ece642rtle_student side)!

A sequence diagram is provided in this folder. Here is a written description of the overview:

ece642rtle_node exists to render the turtle, and accept moves from ece642rtle_student. It also passes bumped() and at_end() information to this node.

ros_turtle_interface starts ece642rtle_student by sending an emtpy message to ece642rtle_node (on turtle1/TRI_req ROS topic)
turtle responds by sending the pose and orientation back (on turtle1/RTI_pob)
ros_turtle_interface calls student code (moveTurtle(...)) with this pose and orientation. Student code may:
- call bumped(...), which prompts a service call on turtle1/RTI_bump
- call atend(...), which prompts a service call on turtle1/RTI_at_end
- call displayVisits, which will update the visit count of the cell that the turtle is at when moveTurtle returns. This is shared on the turtle1/TRI_vist topic
if moveTurtle returns true, the student has updated the position and/or orientation and ros_turtle_interface sends the new information on turtle1/TRI_pob. Otherwise, it sends an empty message on turtle1/TRI_req
Whenever turtle receives a request (turtle1/TRI_req) or pose (turtle1/TRI_pob), it renders the turtle (along with any updated visit count in the cell) and sends back the pose and orientation on turtle1/RTI_pob, starting the cycle over.

**** Mazefile Specification: ****

All but the first two or three lines of the mazefile are of the format
x1,y1,x2,y2
and describe wall segments with endpoints (x1,y1) and (x2,y2).
Because these wall segments are parallel to the x- or y- axis, they must satisfy
x1 == x2 || y1 == y2

The first line can optionally be "RAND_DIR" (see m10.maze). In this case, the turtle will face a random direction when it spawns.

The next two lines specify the start and end cells of the maze, respectively. They are also of the form
x1,y1,x2,y2
Where (x1,y1) is the upper-left and (x2,y2) the lower-right corner of the cell.

I generate the maze by:
- Making the maze (12x12) at http://www.mazegenerator.net/
- Downloading the maze as an SVG
- Removing headers and footers, so that the only things in the file look like
<line x1="2" y1="2" x2="146" y2="2" />
- Using the python script maze_to_points.py to convert that file to our mazefile format (change the first line of the python script, or change it to accept a filename as an argument...)
- Manually adding the start and end cell coordinates, and removing/adding walls at will

**** Notes on runtime monitoring: ****

ros_monitor_interface.cpp implements a basic message listener that passes ROS messages on to the functions tickInterrupt, poseInterrupt, visitInterrupt, bumpInterrupt, and atEndInterrupt. These should be implemented by each individual monitor.

Any time a service is called, ros_turtle_interface sends a ros message that echos the contents (arguments and response) of the service. This is used solely by the monitor. The timestamp of this echo is set to the time of the request, not of the response (this is because the monitors we ask the students to implement check for request timing).

Note that tickInterrupt happens whenever turtle (of ece642rtle_node) sends a pose out. This is consistent with the sequence diagram of communication between nodes: every time ece642rtle_node sends a message on the turtle1/RTI_pob topic, ros_turtle_interface calls moveTurtle.
