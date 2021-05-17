"""particleFilterWalls controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random

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

# Maze configuration, if any of the NESW are set to 1, there
# is either an inner or outer wall there
#[Visited, North, East, South, West]
grid_maze = [[0, 1, 0, 1, 1], [0, 1, 0, 1, 0], [0, 1, 0, 0, 0], [0, 1, 1, 0, 0],
             [0, 1, 0, 0, 1], [0, 1, 1, 0, 0], [0, 0, 1, 0, 1], [0, 0, 1, 0, 1], 
             [0, 0, 1, 0, 1], [0, 0, 0, 1, 1], [0, 0, 1, 1, 0], [0, 0, 1, 0, 1],
             [0, 0, 0, 1, 1], [0, 1, 0, 1, 0], [0, 1, 0, 1, 0], [1, 0, 1, 1, 0]]                 
# noise for measurement and motion models
measurement_noise = 0.25
motion_noise = 0.10
                 
class Particle:
    def __init__(self, n, dir, importance):
        self.n = n
        self.dir = dir
        self.importance = importance
    
    def __repr__(self):
        return f"Particle: cell: {self.n}, direction: {self.dir}, importance: {self.importance}" 

 
# list for the 80 particles
particles = []


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


def move(inches, timestep, dest=None):
    seconds = get_time(inches, max_speed)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update the robot
        update_robot()
        #if dest != None:
        #    print(f"Moving towards cell: {dest}...")
        #print(f"Moving {inches} inches forward...")
        # get the walls around the robot and add them to the map
        if robot.getTime() < end_time:
            leftMotor.setVelocity(max_speed/wheel_radius)
            rightMotor.setVelocity(max_speed/wheel_radius)
            if dest != None and robot_pose[2] == dest:
                grid_maze[robot_pose[2]-1][0] = 1 
        else:
            stop_motors()
            break


def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    #print("Motors stopped.")


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
        #print(f"Rotating {direction}...")
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break    

            
def turn_left(ts, degrees=-90.5):
    rotate(degrees, 1.5, ts, "left")           
            
            
def turn_right(ts, degrees=90.5):
    rotate(degrees, 1.5, ts, "right")
    

def check_if_robot_should_stop():
    # go through all the cells to see if it has a 0,
    # a cell that has not been visited yet, in the list or not
    for el in grid_maze:
        # if there is even a single 0 in the list, there is a cell
        # that hasn't been visited to yet
        if el[0] == 0:
            # there is a cell yet to be visited, 
            # robot should keep going, don't stop
            return False
    # all the cells were 1's at this point, so all cells have been 
    # visited, robot should stop
    return True      


def update_robot(rotating=False):
    # Indicated as (x,y,n,q), where “x,y” represents
    # the robot position in global coordinates, “n” represents 
    # the grid cell number, and “q ” represents the robot global
    # orientation with respect to the global frame of reference,
    # e.g. Pose = (11.5, 2.3, 8, 1.1).
    global robot_pose
    
    # get and print the new measurements
    vals = get_p_sensors_vals()
    
    if not rotating:
        x, y = get_robot_x_y(vals)
        n = get_current_grid_cell(x, y)
    else:
        x = robot_pose[0]
        y = robot_pose[1]
        n = robot_pose[2]
    q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
    # update the robot's pose
    robot_pose = [x, y, n, q]
    # print the last and current positions
    #print(f"Last position: {last_pos}")
    #print(f"Current position: {new_pos}")
    # print new robot pose
    
    #update the direction the robot is facing and print it
    update_direction()
    
def print_robot():
    # print the time
    print(80*"-")
    print(f"Time: {robot.getTime()}")
    # print the visited cells
    print_maze(grid_maze)
    # print the number of particles in every cell
    print_particle_counts()
    print(f"Pose: {robot_pose}")
    print(f"Robot current grid cell: {robot_pose[2]}, Direction: {dir}")
    particle_counts = get_particle_counts()
    max_particles = max(particle_counts)
    max_cell = 0
    for i in range(len(particle_counts)):
            if particle_counts[i] == max_particles:
                max_cell = i + 1
    print(f"From particles, robot thinks it is in cell: {max_cell}")


def get_robot_x_y(vals):
    global last_pos
    global new_pos
    global last_vals
    diff = [vals[0] - last_vals[0], vals[1] - last_vals[1]]
    for i in range(len(diff)):
        diff[i] = math.fabs(diff[i])
    
    if math.fabs(diff[0]) >= .3 or math.fabs(diff[1]) >= .3:
            diff[0] = 0.3
            diff[1] = 0.3
            #print(f"new diff: {diff}")
            
    # diff average of the left and right wheel
    diff_avg = (diff[0]+ diff[1]) / 2.0
    
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
    

def update_direction():
    global dir
    dir = get_direction(robot_pose[3]) 


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


def get_available_turns(walls):
    # 0, means not available, 1 means available
    available_turns = [0, 0, 0]
    if walls[0] == 0 and dir == "North" and robot_pose[2] > 4:
         if grid_maze[robot_pose[2]-5][0] == 0:
            available_turns[0] = 1
    if walls[0] == 0 and dir == "South" and robot_pose[2] < 13:
        if grid_maze[robot_pose[2]+3][0] == 0:
            available_turns[0] = 1    
    if walls[1] == 0 and dir == "North" and robot_pose[2] % 4 != 1:
        if grid_maze[robot_pose[2]-2][0] == 0:
            available_turns[1] = 1
    if walls[2] == 0 and dir == "North" and robot_pose[2] % 4 != 0: 
        if grid_maze[robot_pose[2]][0] == 0:
            available_turns[2] = 1
    return available_turns


"""
measurement model
Importance Factor
Resample
move
motion model
repeat
"""
def traverse_maze(ts):
    global grid_maze
    # direction robot faced before moving
    control = "North"
    while robot.step(ts) != -1:
        walls = get_walls_around_robot()
        available = get_available_turns(walls)
        #add_walls_to_map(walls)
        #print(f"Robot sees these walls: {walls}")
        #print(f"Robot's available turns: {available}")
         # get the particles probabilities for every particle
        measurement_estimation(walls)
        #print_particles()
        # check here for particles before the robot spins or moves
        weights = get_cell_weights()
        #print(f"weights: {weights}")
        norm_weights = normalize_weights(weights)
        #print(f"norm_weights: {norm_weights}")
        resample_particles(norm_weights)
        
        if dir == "North" and available[1] == 1:
            # face left
            turn_left(ts)
        elif dir == "North" and available[2] == 1:
            # face right
            turn_right(ts)
        elif dir == "North" and available[0] == 0 \
            and available[1] == 0 and available[2] == 0:
            # turn around
            turn_right(ts, 91.25) 
            turn_right(ts, 91.25)
            
        if dir == "South":
            walls = get_walls_around_robot()
            available = get_available_turns(walls)
            #add_walls_to_map(walls)
            #print(f"walls: {walls}")
            #print(f"available: {available}")
            if available[0] == 0 and available[1] == 0 \
            and available[2] == 0:
                turn_left(ts)
        # update the control variable
        control = dir     
        # move to the next square
        move(10, ts, None)
        # visited a cell, mark it as visited
        grid_maze[robot_pose[2]-1][0] = 1
        
        if dir != "North":  
            if dir == "West":
                # turn right to face north again
                turn_right(ts)
            elif dir == "South":
                if available[0] == 0 and available[1] == 0 \
                and available[2] == 0:
                    turn_left(ts)
                    move(10, ts)
                    # visited a cell, mark it as visited
                    grid_maze[robot_pose[2]-1][0] = 1
                else:
                    # turn right twice to face north again
                    turn_right(ts)
                    turn_right(ts)
            elif dir == "East":
                # turn left to face north again
                turn_left(ts)   
        motion_update(control)
        #print(f"updated particles after motion control: {get_particle_counts()}" )
        print_robot()
        if check_if_robot_should_stop():        
            break     
    

def make_particles(num_particles):
    global particles
    importance = 1/float(num_particles)
    n = 1
    for i in range(num_particles):
        if i % 5 == 0 and i != 0:
            n += 1
        particles.append(Particle(n, "North", importance))
    

def normalize_particle_importances():
    global particles
    probs = [particles[i].importance for i in range(len(particles))]
    norm_sum = sum(probs)
    #print(f"sum of particles probabilities: {norm_sum}")
    for i in range(len(particles)):
        particles[i].importance /= norm_sum
    #norm_probs = [particles[i].importance for i in range(len(particles))]    
    #print(f"sum of particles probabilities: {sum(norm_probs)}")


def measurement_estimation(state_walls):
    global particles
    # state_walls is are the walls that the robot can see
    # front probability, left p, and right p values
    # determined by measurement model
    fp = 0.0
    lp = 0.0
    rp = 0.0
    # print_particles()
    for i in range(len(particles)):
        #grid_maze is VNESW
        # if there is a front wall
        if grid_maze[particles[i].n-1][1] == state_walls[0]:
            fp = 1.0 - measurement_noise
        else:  # no front wall
            fp = measurement_noise
        
         # if there is a left wall
        if grid_maze[particles[i].n-1][4] == state_walls[1]:
            lp = 1.0 - measurement_noise
        else: # no left wall
            lp = measurement_noise
            
         # if there is a right wall
        if grid_maze[particles[i].n-1][2] == state_walls[2]:
            rp = 1.0 - measurement_noise
        else:  # no right wall
            rp = measurement_noise
        #print(f"i: {i}, fp: {fp}, lp: {lp}, rp: {rp}")
        # importance is the multiplication of fp, lp, rp
        particles[i].importance = fp * lp * rp
    # normalize the importance factors for the particles
    normalize_particle_importances()
    
    
def get_cell_weights():
    weights = [0]*16
    for i in range(len(particles)):
        weights[particles[i].n-1 ] += particles[i].importance
    return weights
    

def normalize_weights(weights):
    norm_weights = [0.0] * len(weights)
    norm = sum(weights)
    for i in range(len(weights)):
        norm_weights[i] = weights[i] / norm
    #print(f"sum of norm weights: {sum(norm_weights)}")
    return norm_weights    
        

def resample_particles(norm_weights):
    global particles
    particle_counts = [0] * 16
    
    # multiply every cell weight by the total number of particles 
    for i in range(len(norm_weights)):
        norm_weights[i] *= len(particles)    
    #print(f"norm_weights * 80: {norm_weights}")
    
    # the particle count for each cell will be the rounded
    # version of the same index in the weights list
    for i in range(len(norm_weights)):
        particle_counts[i] = round(norm_weights[i])
    #print(f"particle_counts: {particle_counts}")
    
    num_particles = sum(particle_counts)
    diff = len(particles) - num_particles
    #print(f"difference in number of particles after resampling: {diff}")
    if diff != 0:
        max_count = max(particle_counts)
        for i in range(len(particle_counts)):
            if particle_counts[i] == max_count:
                particle_counts[i] += diff
                break
    # alter the n values for the particles and importance 
    # values of the particles   
    ind = 0
    for i in range(len(norm_weights)):
        for j in range(particle_counts[i]):
            particles[ind].n = i+1
            particles[ind].importance =  norm_weights[i] / float(particle_counts[i] * len(particles))
            ind += 1
        #print(f"i: {i}, particle_counts[i]: {particle_counts[i]}")
    #print("end of resample:")
    #print_particles()
    
      
def motion_update(control):
    global particles
    motion_amount = 0.0
    # counts of particles in each cell
    particle_counts = get_particle_counts()
    # motion update
    # move all of the particles the direction of the control
    if control == "North":
        motion_amount = -4   
    elif control == "East": 
        motion_amount = 1
    elif control == "South":
        motion_amount = 4
    elif control == "West": 
        motion_amount = -1
    
    # list to hold if a control motion is possible based on 
    # a cell's state from the map    
    # grid_maze is VNESW
    motion_possible = [False] * len(particles)
    for i in range(len(particles)):
        if control == "North":
            if grid_maze[particles[i].n-1][1] == 0:
                motion_possible[i] = True
        if control == "East":
            if grid_maze[particles[i].n-1][2] == 0:
                motion_possible[i] = True
        if control == "South":
            if grid_maze[particles[i].n-1][3] == 0:
                motion_possible[i] = True
        if control == "West":
            if grid_maze[particles[i].n-1][4] == 0:
                motion_possible[i] = True
    #print(f"motions: {motion_possible}")
    
    for i in range(len(particles)):
        # the random number should simulate the effect of \
        # keeping the number of those cells in that spot times
        # the motion_noise in the same spot and the rest should
        # move to the according spot if the motion is possible
        if motion_possible[i]:
            rand = random.uniform(0,1)
            if rand > motion_noise: 
                particles[i].n += motion_amount 
            # otherwise the particles n stays the same    
    #print("motion updated:")
    #print_particles()
    

def print_particles():
    for i in range(len(particles)):
        print(f"{i}: {particles[i]}")
    
    
def get_particle_counts():
    counts = [0] * 16
    for i in range(len(particles)):    
        counts[particles[i].n-1] += 1
    return counts

        
def print_particle_counts():
    counts = get_particle_counts()
    print("particle counts:")
    print(f"{counts[0:4]}")
    print(f"{counts[4:8]}")
    print(f"{counts[8:12]}")
    print(f"{counts[12:16]}")
    #print(f"count of particles; {sum(counts)}")
    

def get_walls_around_robot():
    # front, left, right
    walls = [0, 0, 0]
    vals = get_d_sensors_vals()
    for i in range(3):
        if vals[i] < 10.0:
            walls[i] = 1
    return walls


"""
def add_walls_to_map(walls):
    global grid_maze
    #[Visited, North, East, South, West]
    if dir == "North":
        grid_maze[robot_pose[2]-1][1] = walls[0] # front is north
        grid_maze[robot_pose[2]-1][4] = walls[1] # left is west
        grid_maze[robot_pose[2]-1][2] = walls[2] # right is east
    # add the added walls to the accompanying cells nearby
    for cell in range(16):
        #[Visited, North, East, South, West]
        # if the current cell and the cell to the right 
        # both share an inner wall, if one cell is set to 
        # 1, set both of them to 1 for their E/W wall
        if cell < 15:
            if grid_maze[cell][2] != grid_maze[cell+1][4]:
                grid_maze[cell][2] = 1
                grid_maze[cell+1][4] = 1
                #print(f"{cell}: A: made a change")
        else:  # cell 15
            if grid_maze[cell][4] != grid_maze[cell-1][2]:
                grid_maze[cell][4] = 1
                grid_maze[cell-1][2] = 1
                #print(f"{cell}: B: made a change")
        # if the current cell and the cell beneath it 
        # both share an inner wall, if one cell is set to 
        # 1, set both of them to 1 for their N/S wall
        if cell < 12:
            if grid_maze[cell][3] != grid_maze[cell+4][1]:
                grid_maze[cell][3] = 1
                grid_maze[cell+4][1] = 1
                #print(f"{cell}: C: made a change")
        else: # cells 13 to 15
            if grid_maze[cell][1] != grid_maze[cell-4][3]:
                grid_maze[cell][1] = 1
                grid_maze[cell-4][3] = 1
                #print(f"{cell}: D: made a change")
"""
        

# maze walls
def print_maze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " +str(maze[x+2][1])
              +"\t  " +str(maze[x+3][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" +str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
              +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
              +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2]) +"  |")
        print("|  "+str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " +str(maze[x+2][3])
              +"\t  " +str(maze[x+3][3])+"    |")
        if(i==3):
            print("|_______________________________________|\n")
        else:
            print("|                                       |")


def main():
    # make the 80 particles
    make_particles(80)
    # print the orginal robot stats
    print_robot()
    # traverse the maze until it has visited every cell
    while robot.step(timestep) != -1:
        traverse_maze(timestep)
        # check after every move to see if the robot should stop or not
        if check_if_robot_should_stop():
            print_robot()
            break                  


if __name__ == "__main__":
    main()
