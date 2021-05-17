"""lab3task1Noise - motionEstimationNoise controller."""

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

# getting the position sensors
lps = robot.getDevice('left wheel sensor')
rps = robot.getDevice('right wheel sensor')
lps.enable(timestep)
rps.enable(timestep)

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
wheel_circ = 2 * 3.14 * wheel_radius
enc_unit = wheel_circ / 6.28
max_speed = 4
# distance between wheels in inches and middle of the robot d_mid
d = 2.28
d_mid = d / 2.0

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
# robot starting position in grid cell 16, middle of cell, 
# it is also the last position for future moves
last_pos = [15.0, -15.0]
# new position to keep track of x,y as the robot keeps moving
new_pos = [15.0, -15.0]
# keep track of the last values from position sensors, so the difference can be added to position
last_vals = [0, 0]
# robot pose to hold x, y, grid number n, and orientation theta(represented as q)
robot_pose = [15.0, -15.0, 16, 180]
# last_n, keeps track of the last grid cell the robot was in to look for changes
last_n = 16
# array to keep track of all non-visited and visited cells, '.'=non-visited, and 'X'=visited
visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', 'X']
# dirs are the counts of each direction between cells based on 
# imu readings, imu_average is determined based on the max count
# and the degree value associated with that direction
dirs = {"North": 0, "West": 0, "East": 0, "South": 0}
imu_average = 0.0

# converts meters to inches
def m_to_i(meters):
    return meters * 39.3701


def pos_s_to_inches(val):
    return math.fabs(val * wheel_radius)
        

def print_measurements():
    print(f"left: {pos_s_to_inches(lps.getValue())}, right: {pos_s_to_inches(rps.getValue())}, imu: {(imu.getRollPitchYaw()[2] * 180) / 3.14159}")
    

def get_p_sensors_vals():
    # returns left and right position sensors
    return pos_s_to_inches(lps.getValue()), pos_s_to_inches(rps.getValue())

    
def get_d_sensors_vals():
    # returns front, left, right sensors in inches
    return m_to_i(fds.getValue()), m_to_i(lds.getValue()), m_to_i(rds.getValue())


def get_time(distance, speed):
    return distance / speed
    

def noise_sol_imu(val):
    global imu_average
    global dir
    dirs[get_direction(val)] += 1
        
    #print(f"dirs: {dirs}")
    mv = max(dirs.values())
    for k in dirs.keys():
        if dirs[k] == mv:
            dir = k
            break
    if dir == "North":
        imu_average = 180
    elif dir == "West":
        imu_average = -90    
    elif dir == "East":
        imu_average = 90
    elif dir == "South":
        imu_average = 0
    return val

def move(inches, timestep, dest):
    seconds = get_time(inches, max_speed)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update the robot
        update_robot()
        print(f"Moving towards cell: {dest}...")
        print(f"Moving {inches} inches forward...")
        
        if robot.getTime() < end_time:
            leftMotor.setVelocity(max_speed/wheel_radius)
            rightMotor.setVelocity(max_speed/wheel_radius)
            if robot_pose[2] == dest:
                visited_cells[robot_pose[2]-1] = 'X' 
            # check after every move to see if the robot should stop or not
            #if check_if_robot_should_stop():
             #   stop_motors()
              #  return True
        else:
            stop_motors()
            return False


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
    global last_vals
    # get the left and right rotaional speeds to turn x degrees in y seconds
    left, right = get_rot_speed_rad(degrees, seconds, wheel_radius, d_mid)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update and print the robot's details
        update_robot(rotating=True)
        # still update the last vals
        vals = get_p_sensors_vals()
        last_vals = vals
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


def update_robot(rotating=False):
    # Indicated as (x,y,n,q), where “x,y” represents
    # the robot position in global coordinates, “n” represents 
    # the grid cell number, and “q ” represents the robot global
    # orientation with respect to the global frame of reference,
    # e.g. Pose = (11.5, 2.3, 8, 1.1).
    global robot_pose
    global dir
    global last_n
    
    # print the time
    print(80*"-")
    print(f"Time: {robot.getTime()}")
    
    # Read the sensors:
    # print the last measurements
    #print(f"last_vals: {last_vals}")
    
    # get and print the new measurements
    vals = get_p_sensors_vals()
    #print_measurements()
     
    # print the visited cells
    print_visited_cells()
    
    if not rotating:
        x, y = get_robot_x_y(vals)
        n = get_current_grid_cell(x, y)
        q = noise_sol_imu((imu.getRollPitchYaw()[2] * 180) / 3.14159)
    else:
        x = robot_pose[0]
        y = robot_pose[1]
        n = robot_pose[2]
        q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
    
    if n != robot_pose[2]:
        last_n = robot_pose[2]
    
    # update the robot's pose
    robot_pose = [x, y, n, q]
    # print the last and current positions
    #print(f"Last position: {last_pos}")
    #print(f"Current position: {new_pos}")
    # print new robot pose
    print(f"Pose: {robot_pose}")
    #update the direction the robot is facing and print it
    if not rotating:
        dir = get_direction(imu_average)
    else:
        dir = get_direction(q)
    #print(f"imu_average: {imu_average}")
    #print(f"Last grid cell: {last_n}")
    print(f"Robot current grid cell: {robot_pose[2]}, Direction: {dir}")


def get_robot_x_y(vals):
    global last_pos
    global new_pos
    global last_vals
    diff = [vals[0] - last_vals[0], vals[1] - last_vals[1]]
    for i in range(len(diff)):
        diff[i] = math.fabs(diff[i])
    #print(f"diff: {diff}")
    
    if math.fabs(diff[0]) >= .3 or math.fabs(diff[1]) >= .3:
            diff[0] = 0.3
            diff[1] = 0.3
            #print(f"new diff: {diff}")
            
    # diff average of the left and right wheel
    diff_avg = (diff[0]+ diff[1]) / 2.0
    #print(f"diff_avg: {diff_avg}")
    
    # x and y are dependent on the direction the robot is moving in
    if dir == "North":
        x = new_pos[0]
        y = new_pos[1] + diff_avg
    elif dir == "West":
        x = new_pos[0] - diff_avg
        y = new_pos[1] 
    elif dir == "East":
        x = new_pos[0] + diff_avg
        y = new_pos[1] 
    elif dir == "South":
        x = new_pos[0] 
        y = new_pos[1] - diff_avg
    # update the old last_pos as the values from the current_pos
    last_pos[0] = new_pos[0]
    last_pos[1] = new_pos[1]
    # update the last vals
    last_vals = vals
    # store the new x and y into the new_pos
    new_pos = x, y
    return x, y


def get_current_grid_cell(x, y):
    n = 0
    row = 0
    col = 0
    
    # how to determine grid row from y
    if y >= -20 and y < -10:
        # the bottom row
        row = 3
    elif y >= -10 and y < 0:
        # row above the bottom row
        row = 2
    elif y >= 0 and y < 10:
        # row beneath the top row
        row = 1 
    elif y >= 10 and y <= 20:
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
    

def get_direction(imu_val):
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
    elif 45 <= imu_val <= 135:
        dir = "East"
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
    return dir 


def move_to_cell(dest, ts):
    global imu_average
    global dirs
    # reset the imu data trackers when robot reaches the dest cell
    # and then restarts here
    dirs = dirs = {"North": 0, "West": 0, "East": 0, "South": 0}
    imu_average = 0.0
    # get the rows and the columns for the destination cell
    dest_row = n_rc[dest-1][0]
    dest_col = n_rc[dest-1][1]
    
    # the robot is on the same row or column as the destination cell
    same_row = False
    same_col = False
    
    # robot is facing correct direction to move to the correct cell->True
    facing = False
    
    while robot.step(ts) != -1:
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
        # if the row from the last grid cell that the robot was in
        # isn't the same as the current grid cell's row, reset the
        # dictionary counts, or when reset when the robot reaches
        if n_rc[last_n-1][0] != n_rc[robot_pose[2]-1][0]:
            dirs = dirs = {"North": 0, "West": 0, "East": 0, "South": 0}
            imu_average = 0.0
        
        # show the state of same_row and same_col
        # print(f"same_row: {same_row}, same_col: {same_col}, facing: {facing}")
        
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


def main():
    # cell to go to, set to current starting cell for now
    cell = 16
    while robot.step(timestep) != -1:
        # get the next cell to visit
        if robot_pose[2] % 2 == 0:
            cell -= 3
        else:
            cell -1
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
            break
    

if __name__ == "__main__":
    main()
