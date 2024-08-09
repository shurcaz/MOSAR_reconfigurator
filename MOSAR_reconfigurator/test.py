from logic_layer.task_planner import Module, State

# A range of system input/outputs to be used for testing purposes

def four_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((0,0,1), Module([1, 1, 1]))          # white
    s_start.insert((1,1,0), Module([0, 1, 1]))          # aqua
    s_start.insert((0,1,0), Module([0, 1, 0]))          # green
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((1,1,1), Module([1, 0, 0]))          # red
    s_goal.insert((0,1,0), Module([1, 1, 1]))          # white
    s_goal.insert((1,0,0), Module([0, 1, 1]))          # aqua
    s_goal.insert((1,1,0), Module([0, 1, 0]))          # green
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "4 modules"


def five_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((1,0,0), Module([0, 1, 0]))          # green
    s_start.insert((2,0,0), Module([0, 0, 1]))          # blue
    s_start.insert((2,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_start.insert((2,1,1), Module([1, 1, 1]))          # white
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((0,0,0), Module([1, 0, 0]))          # red
    s_goal.insert((1,0,0), Module([0, 1, 0]))          # green
    s_goal.insert((0,1,0), Module([0, 0, 1]))          # blue
    s_goal.insert((1,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_goal.insert((2,1,0), Module([1, 1, 1]))          # white
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "5 modules"


def six_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((1,0,0), Module([0, 1, 0]))          # green
    s_start.insert((2,0,0), Module([0, 0, 1]))          # blue
    s_start.insert((2,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_start.insert((2,1,1), Module([1, 1, 1]))          # white
    s_start.insert((0,1,0), Module([1, 1, 1]))          # white
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((0,0,0), Module([1, 0, 0]))          # red
    s_goal.insert((1,0,0), Module([0, 1, 0]))          # green
    s_goal.insert((0,1,0), Module([0, 0, 1]))          # blue
    s_goal.insert((1,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_goal.insert((0,1,1), Module([1, 1, 1]))          # white
    s_goal.insert((2,1,0), Module([1, 1, 1]))          # white
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "6 modules"


def seven_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((1,0,0), Module([0, 1, 0]))          # green
    s_start.insert((2,0,0), Module([0, 0, 1]))          # blue
    s_start.insert((2,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_start.insert((2,1,1), Module([1, 1, 1]))          # white
    s_start.insert((0,1,0), Module([1, 1, 1]))          # white
    s_start.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((0,0,0), Module([1, 0, 0]))          # red
    s_goal.insert((1,0,0), Module([0, 1, 0]))          # green
    s_goal.insert((2,1,0), Module([0, 0, 1]))          # blue
    s_goal.insert((1,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_goal.insert((1,1,1), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,0), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "7 modules"


def eight_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((1,0,0), Module([0, 1, 0]))          # green
    s_start.insert((2,0,0), Module([0, 0, 1]))          # blue
    s_start.insert((2,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_start.insert((2,1,1), Module([1, 1, 1]))          # white
    s_start.insert((0,1,0), Module([1, 1, 1]))          # white
    s_start.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_start.insert((2,2,0), Module([0, 1, 1]))          # aqua
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((0,0,0), Module([1, 0, 0]))          # red
    s_goal.insert((1,0,0), Module([0, 1, 0]))          # green
    s_goal.insert((2,1,0), Module([0, 0, 1]))          # blue
    s_goal.insert((1,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_goal.insert((1,1,1), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,0), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_goal.insert((2,0,0), Module([0, 1, 1]))         # aqua
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "8 modules"


def nine_mod_config():
    s_start = State()
    s_start.insert((0,0,0), Module([1, 0, 0]))          # red
    s_start.insert((1,0,0), Module([0, 1, 0]))          # green
    s_start.insert((2,0,0), Module([0, 0, 1]))          # blue
    s_start.insert((2,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_start.insert((2,1,1), Module([1, 1, 1]))          # white
    s_start.insert((0,1,0), Module([1, 1, 1]))          # white
    s_start.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_start.insert((2,2,0), Module([0, 1, 1]))          # aqua
    s_start.insert((0,2,0), Module([1, 0, 1]))          # pink
    s_start.display("Starting Configuration")
    
    s_goal = State()
    s_goal.insert((0,0,0), Module([1, 0, 0]))          # red
    s_goal.insert((1,0,0), Module([0, 1, 0]))          # green
    s_goal.insert((2,1,0), Module([0, 0, 1]))          # blue
    s_goal.insert((1,1,0), Module([0.5 ,0.5, 0.5]))    # grey
    s_goal.insert((1,1,1), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,0), Module([1, 1, 1]))          # white
    s_goal.insert((0,1,1), Module([1, 1, 0]))          # yellow
    s_goal.insert((2,0,0), Module([0, 1, 1]))          # aqua
    s_goal.insert((0,2,0), Module([1, 0, 1]))          # pink
    s_goal.display("Goal Configuration")
    return s_start, s_goal, "9 modules"


def test2():
    # Starting State
    s_start = State()
    for x in range(0,3):
        for y in range(0,3):
            for z in range(0,2):
                if ([x,y,z] != [1,1,0]):
                    s_start.insert((x,y,z), Module([1, 0, 0]))          # red
                    
    s_start.insert((1,1,0), Module([1, 1, 1]))                          # white
    s_start.display("Starting Configuration")
    
    # Goal State
    s_goal = State()
    for x in range(0,3):
        for y in range(0,3):
            for z in range(0,2):
                if ([x,y,z] != [1,1,1]):
                    s_goal.insert((x,y,z), Module([1, 0, 0]))          # red
                    
    s_goal.insert((1,1,1), Module([1, 1, 1]))          # red
    s_goal.display("Goal Configuration")
    
    return s_start, s_goal, "T2 - Box Escape"


def test3():
    s_start = State()
    for x in range(1,6,2):
        for y in range(1,4,2):
            z = 1
            s_start.insert([x,y,z], Module([0.5 ,0.5, 0.5]))        # grey
            s_start.insert([x+1,y+1,z], Module([0.5 ,0.5, 0.5]))    # grey
            s_start.insert([x+1,y,z], Module([1, 0, 0]))            # red
            s_start.insert([x,y+1,z], Module([1, 0, 0]))            # red
            
            z = 2
            s_start.insert([x,y,z], Module([1, 0, 0]))              # red
            s_start.insert([x+1,y+1,z], Module([1, 0, 0]))          # red
            s_start.insert([x+1,y,z], Module([0.5 ,0.5, 0.5]))      # grey
            s_start.insert([x,y+1,z], Module([0.5 ,0.5, 0.5]))      # grey
            
    x = 6
    for y in range(1,5):
        for z in range(1,3):
            s_start.remove_module([x,y,z])
    
    for x in range(1,6):
        for y in range(2,5):
            #for z in range(1,3):
            z = 3
            s_start.insert([x,y,z], Module([1, 1, 1]))               # white
    
    for x in range(1,6):
        for z in range(1,3):
            s_start.insert([x,5,z], Module([1, 1, 1]))               # white
            
                
    s_start.display()
              
                    
    s_goal = State()
    # Add legs
    def add_chair_leg(x_offset, y_offset):
        for x in range(1,2):
            for y in range(1,2):
                for z in range(1,6):
                    s_goal.insert([x + x_offset,y + y_offset,z], Module([1, 0, 0]))           # red
    add_chair_leg(0, 0)
    add_chair_leg(4, 0)
    add_chair_leg(0, 4)
    add_chair_leg(4, 4)
    
    # Add Base
    for x in range(1,6):
        for y in range(1,6):
            s_goal.insert([x,y,6], Module([1, 1, 1]))               # white
                
    # Add back
    x = 1
    for y in range(1,6):
        for z in range(7,13):
            s_goal.insert([x,y,z], Module([0.5 ,0.5, 0.5]))         # grey
    
    # create slits
    x = 1
    y = 2
    for z in range(7,12):
        s_goal.remove_module([x,y+2,z])
        s_goal.remove_module([x,y,z])
        
    s_goal.display()
                  
    return s_start, s_goal, "T3 - Chair Construction"


def test4():
    s_start = State()
    for x in range(0,5):
        for y in range(0,4):
                s_start.insert([x,y,0], Module([160/255, 32/255, 240/255]))
                s_start.insert([x,y,1], Module([0.5, 0.5, 0.5]))
    
    s_start.insert([1,1,2], Module([0.5, 0.5, 0.5]))
    
    s_start.display("Starting Configuration")
    
    s_goal = State()
    for x in range(0,5):
        for y in range(0,5):
                s_goal.insert([x,y,0], Module([0.5, 0.5, 0.5]))
                s_goal.insert([x,y,1], Module([160/255, 32/255, 240/255]))
              
    s_goal.insert([4,2,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([2,0,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([0,2,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([2,4,2], Module([160/255, 32/255, 240/255]))
    
    s_goal.insert([1,0,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([0,1,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([3,4,2], Module([160/255, 32/255, 240/255]))
    s_goal.insert([4,3,2], Module([160/255, 32/255, 240/255]))
                
    for x in range(1,4):
        for y in range(1,4):
            s_goal.remove_module([x,y,1])
            
    s_goal.remove_module([0,0,0])
    s_goal.remove_module([0,0,1])
    s_goal.remove_module([0,4,0])
    s_goal.remove_module([0,4,1])
    s_goal.remove_module([4,0,0])
    s_goal.remove_module([4,0,1])
    s_goal.remove_module([4,4,0])
    s_goal.remove_module([4,4,1])
                
    s_goal.display("Goal Configuration")
    
    return  s_start, s_goal, "T4 - Crown Construction"