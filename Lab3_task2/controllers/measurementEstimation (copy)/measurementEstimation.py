"""measurementEstimation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#enable distance sensors
fds = robot.getDevice('front_ds')
lds = robot.getDevice('left_ds')
rds = robot.getDevice('right_ds')
fds.enable(timestep)
lds.enable(timestep)
rds.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# values for robot
wheel_radius = 1.6 / 2.0
kp = 1.0
max_speed = 4

# world values
# n is index+1 of the associated (row,column) combinations below
# for example, (0,0) is row=0, column=0, so n would be index of 
# this in the list, which is 0, so add 1, so n=0+1=1
n_rc = [(0,0), (0,1), (0,2), (0,3),
        (1,0), (1,1), (1,2), (1,3),
        (2,0), (2,1), (2,2), (2,3),
        (3,0), (3,1), (3,2), (3,3)
       ]

# state of robot
dir = "North"
robot_pose = [0, 0, 16, 0]
visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', 'X']

# converts meters to inches
def m_to_i(meters):
    return meters * 39.3701
    

def print_measurements():
    print(f"front: {m_to_i(fds.getValue())}, left: {m_to_i(lds.getValue())}, right: {m_to_i(rds.getValue())}, imu: {(imu.getRollPitchYaw()[2] * 180) / 3.14159}")
    
    
def get_d_sensors_vals():
    # returns front, left, right sensors in inches
    return m_to_i(fds.getValue()), m_to_i(lds.getValue()), m_to_i(rds.getValue())


def get_time(distance, speed):
    return distance / speed


def move(inches, timestep, dest):
    seconds = get_time(inches, max_speed)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # print the robot's details
        print_robot()
        print(f"Moving towards cell: {dest}...")
        print(f"Moving {inches} inches forward...")
        
        if robot.getTime() < end_time:
            leftMotor.setVelocity(max_speed/wheel_radius)
            rightMotor.setVelocity(max_speed/wheel_radius)
        else:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break


def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("Motors stopped.")


def get_rot_speed_rad(degrees, seconds, wheel_radius, d_mid):
    circle = d_mid * 2 * math.pi
    dist = (degrees / 360) * circle
    linear_vel = dist / seconds
    left_wheel_speed = linear_vel / wheel_radius
    right_wheel_speed = -1 * linear_vel / wheel_radius
    return left_wheel_speed, right_wheel_speed

  
def rotate(degrees, seconds, timestep, direction):
     # wheel radius in inches
    wheel_radius = 1.6 / 2.0
    # distance between wheels in inches and middle of the robot d_mid
    d = 2.28
    d_mid = d / 2.0
    
    # get the left and right rotaional speeds to turn x degrees in y seconds
    left, right = get_rot_speed_rad(degrees, seconds, wheel_radius, d_mid)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # print the robot's details
        # print the time
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        
        # Read the sensors:
        vals = get_d_sensors_vals()
            
        # Process sensor data here.
        # print the visited cells and update and print the robot pose
        print_visited_cells()
        update_robot_pose(vals)
            
        #update the direction the robot is facing and print it
        update_direction()      
        print(f"Rotating {direction}...")
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break    

            
def turn_left(ts):
	rotate(-90.5, 1.5, ts, "left")           
            
            
def turn_right(ts):
	rotate(90.5, 1.5, ts, "right")


def print_visited_cells():
    pr_vc = ""
    for i in range(len(visited_cells)):
        if i % 4 == 0 and i != 0:
            pr_vc += "\n"
        pr_vc += visited_cells[i]
    print("Visited cells:")
    print(pr_vc)
    

def check_if_robot_should_stop():
    # go through all the cells to see if it has a '.',
    # a cell that has not been visited yet, in the list or not
    for el in visited_cells:
        # if there is even a single '.' in the list, there is a cell
        # that hasn't been visited to yet
        if el == '.':
            # there is a cell yet to be visited, 
            # robot should keep going, don't stop
            return False
    # all the cells were 'X's at this point, so all cells have been 
    # visited, robot should stop
    return True      


def update_robot_pose(vals):
    # Indicated as (x,y,n,q), where “x,y” represents
    # the robot position in global coordinates, “n” represents 
    # the grid cell number, and “q ” represents the robot global
    # orientation with respect to the global frame of reference,
    # e.g. Pose = (11.5, 2.3, 8, 1.1).
    global robot_pose
    x, y = get_robot_x_y(vals)
    n = get_current_grid_cell(x, y)
    q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
    # update the robot's pose
    robot_pose = [x, y, n, q]
    print(f"Pose: {robot_pose}")


def get_robot_x_y(vals):
    if dir == "North":
        x = 20 - vals[2]
        y = 20 - vals[0]
    elif dir == "West":
        x = vals[0] - 20
        y = 20 - vals[2]
    elif dir == "East":
        x = 20 - vals[0]
        y = vals[2] - 20
    elif dir == "South":
        x = vals[2] - 20
        y = vals[0] - 20
    return x, y


def get_current_grid_cell(x, y):
    n = 0
    row = 0
    col = 0
    
    # how to determine grid row from y
    if y >= -20 and y <= -10:
        # the bottom row
        row = 3
    elif y > -10 and y <= 0:
        # row above the bottom row
        row = 2
    elif y > 0 and y <= 10:
        # row beneath the top row
        row = 1 
    elif y > 10 and y <= 20:
        # the top row
        row = 0 
        
    # how to determine grid column from x
    if x >= -20 and x <= -10:
        # the left column
        col = 0
    elif x > -10 and x <= 0:
        # the middle left column
        col = 1
    elif x > 0 and x <= 10:
        # the middle right column
        col = 2 
    elif x > 10 and x <= 20:
        # the right column
        col = 3 
        
    # get the n based off of the row, column combination
    for i in range(len(n_rc)):
        if n_rc[i][0] == row and n_rc[i][1] == col:
            # grid cell is 1 plus index of row,col combination
            n = i + 1
            break
    # print(f"Row: {row} Column: {col} N: {n}")
    return n
    

def update_direction():
    global dir
    if (robot_pose[3] <= -165 and robot_pose[3] >= -180) or (165 <= robot_pose[3] <= 180):
        dir = "North"
    elif robot_pose[3] <= -75 and robot_pose[3] >= -105:
        dir = "West"
    elif 75 <= robot_pose[3] <= 105:
        dir = "East"
    elif (-15 <= robot_pose[3] <= 0) or (0 <= robot_pose[3] <= 15):
        dir = "South"
    print(f"Dir: {dir}")   


def move_to_cell(dest, ts):
    # get the rows and the columns for the destination cell
    dest_row = n_rc[dest-1][0]
    dest_col = n_rc[dest-1][1]
    
    # the robot is on the same row or column as the destination cell
    same_row = False
    same_col = False
    
    # robot is facing correct direction to move to the correct cell
    facing = False
    
    while robot.step(ts) != -1:
        # print the robot's details
        print_robot()
        
        # determine if same row
        if dest_row == n_rc[robot_pose[2]-1][0]:
            same_row = True
        else:
            same_row = False
            
        # determine if same_col
        if dest_col == n_rc[robot_pose[2]-1][1]:
            same_col = True
        else:
            same_col = False
        
        # show the state of same_row and same_col
        print(f"same_row: {same_row}, same_col: {same_col}, facing: {facing}")
        
        # if not on the same row, move the robot to the same row
        if not same_row:
            move(10, ts, dest)
            # visited a cell, mark it as visited
            visited_cells[robot_pose[2]-1] = 'X'
        
        # if the robot moved to the same row and still needs to move to the same column
        if same_row and not same_col:
            if not facing:
                if dest < robot_pose[2]:
                    turn_left(ts)
                else:
                    turn_right(ts) 

                # now the robot is facing the correct direction
                facing = True
            else:
                move(10, ts, dest)
                # visited a cell, mark it as visited
                visited_cells[robot_pose[2]-1] = 'X'
        
        if same_row and same_col:
            if dir == "West":
                # turn right to face north again
                turn_right(ts)
            elif dir == "South":
                # turn right twice to face north again
                turn_right(ts)
                turn_right(ts)
            elif dir == "East":
                # turn left to face north again
                turn_left(ts)
        
            # made it to the cell, mark it as visited
            visited_cells[dest-1] = 'X'
            break
            

def print_robot():
    # print the time
    print(80*"-")
    print(f"Time: {robot.getTime()}")
    
    # Read the sensors:
    #print_measurements()
    vals = get_d_sensors_vals()
        
    # Process sensor data here.
    # print the visited cells and update and print the robot pose
    print_visited_cells()
    update_robot_pose(vals)
    print(f"Robot current grid cell: {robot_pose[2]}")
        
    #update the direction the robot is facing and print it
    update_direction()      


def main():
    # next cell to go to
    cell = robot_pose[2]
    while robot.step(timestep) != -1:
        # print the robot's details
        print_robot()
    
        # get the next cell to visit
        cell -= 1
        if visited_cells[cell] == 'X':
            # the next cell to be visited already has been, make things
            # go faster by going to the next non-visited cell
            for i in range(len(visited_cells)):
                if i != 0 and visited_cells[i] == 'X' and visited_cells[i-1] == '.':
                    cell = i
                    break
        # visit the cell
        move_to_cell(cell, timestep)
        
        # check after every move to see if the robot should stop or not
        if check_if_robot_should_stop():
            # print the robot's stats one last time and then end the while loop
            print_robot()
            break
    

if __name__ == "__main__":
    main()
