classdef Robot < handle
   %{
   Class to manage robot functions, attributes, and
   maze exploration logic. Requires our MazeGraph.m
   class.
   The Robot will explore/scan the maze in a DFS-like
   manner until it reaches the pickup. Then if the robot
   knows the dropoff spot, it uses a DFS-like search
   to produce a sequence of instructions to get there.
   If it does not, it will continue to search for it.
   When it finds it, it will use the DFS-like search
   to produce a sequence of instructions to get to
   the start.
   Assumes:
   Sensors:
    port 1: color
    port 2:
    port 3: ultrasonic dist
    port 4:
   Movement Motors:
    Port A
    Port B
   Zones/Colors:
    red = stop for 2s
    blue (2) = pickup
    yellow (4) = dropoff
    green (3) = start
   %}
   properties
       brick
       % for run() logic
       pickupKnown    
       dropoffKnown    
       pickedUp        
       droppedOff
       allDone
       % coordinate representation of position in matrix
       y
       x
       % Stores 2D Matrix representations of Maze Properties
       g   % See MazeClass
       % Movement Attributes
       scanDist        % calibrate for appropriate movement
       movePause       % calibrate for appropriate scanning
       rotatePause     % calibrate for appropriate rotation
       orientation     % 0='north', 1='west', 2='south, 3='east'
                       % "north" is the "top" of the matrix representation
                       % of the maze.
       moveStack       % list, manages the moves made during "exploration"
                       % to enable DFS-like search
       moveList        % list, manages the "unscanned" neighbors of a node
   end
                                                 
   methods
       function obj = Robot(brick)
           % Initialize: robot = Robot(brick);
           if nargin == 1
               obj.brick = brick;
           end
           obj.pickupKnown = 0;
           obj.dropoffKnown = 0;
           obj.pickedUp = 0;
           obj.droppedOff = 0;
           obj.allDone = 0;
           % Determine starting position in matrix before initializaiton:
           y = 1;
           x = 1;
           % Initialize MazeGraph based on the size of the maze
           % and the starting position.
           % Our maze is can be represented by a 3x6 matrix. We
           % started at [1, 1] facing north.
           obj.g = MazeGraph(3, 6, y, x);
           % Initialize Movement Constants and Attributes
           obj.scanDist = 30;      % UPDATE FOR CALIBRATION
           obj.movePause = .7;     % UPDATE FOR CALIBRATION
           obj.rotatePause = 2.56; % UPDATE FOR CALIBRATION
           obj.orientation = 0;
           obj.moveStack = [];
           obj.moveList = [];
          
       end
      
       function dist = getDist(obj) % returns sensor's distance
           dist = double(obj.brick.UltrasonicDist(3));
       end
       function obj = rotateL90(obj)
           % Rotate the object left 90 and update the
           % Orientation
           obj.orientation = mod(obj.orientation+1,4);
           disp(obj.orientation);
           obj.brick.MoveMotor('A',30);
           obj.brick.MoveMotor('B',-30);
           pause(obj.rotatePause);
           obj.brick.StopMotor('AB', 'Brake');
       end
       function obj = rotateR90(obj)
           obj.orientation = mod(obj.orientation-1,4);
           disp(obj.orientation);
           obj.brick.MoveMotor('A',-30);
           obj.brick.MoveMotor('B',30);
           pause(obj.rotatePause);
           obj.brick.StopMotor('AB', 'Brake');
       end
       function obj = orient(obj, b) % b is desired orientation
       % Decides how to move from current orientation to
       % desired orientation.
       % Simplifies the problem with some sneaky logic
       % inferred from a pattern within the possibilities.
           a = obj.orientation; % a is current orientation
           if mod(a-b, 2) == 0 % if a-b is even:
               if abs(a-b) > 0 % if a-b is not 0, rotate 180
                   obj.rotateL90();
                   obj.rotateL90();
               end
               % (if it is 0, stay put)
           else % elseif a-b is odd
               if (a-b) == 3 || (a-b) == -1 % if this is true
                   obj.rotateL90(); % then turn left
               else
                   obj.rotateR90(); % otherwise turn right
               end
           end
       end
       function obj = scan(obj) % Discover the surroundings of currNode
           obj.moveList = [];   % keeps track of the direcitions that are
                                % unexplored
           pause(.1);
           dist = obj.getDist();
           fprintf('distance read to north: %f\n', dist);
           % If we can (and have yet to) move north:
           if dist > obj.scanDist && obj.g.checkUp(obj.y, obj.x) == 0
               % set the north node in matrix to 1, and
               % set the Up edge representation to 1
               obj.g.setUp(obj.y, obj.x, 1);  % update edge representation
               obj.g.addNode(obj.y-1, obj.x); % update node representation
               disp('added node to north');
               obj.moveList = [obj.moveList, 0];
           end
           obj.rotateL90();
           pause(.1);
           dist = obj.getDist();
           fprintf('distance read to west: %f\n', dist);
           % If we can (and have yet to) move west:
           if dist > obj.scanDist && obj.g.checkLeft(obj.y, obj.x) == 0
               % set the west node in matrix to 1, and
               % set the Left edge representation to 1
               obj.g.setLeft(obj.y, obj.x, 1);
               obj.g.addNode(obj.y, obj.x-1);
               disp('added node to west');
               obj.moveList = [obj.moveList, 1];
           end
           obj.rotateL90();
           pause(.1);
           dist = obj.getDist();
           fprintf('distance read to south: %f\n', dist);
           % If we can (and havent yet) move south:
           if dist > obj.scanDist && obj.g.checkDown(obj.y, obj.x) == 0
               % set the south node in matrix to 1, and
               % set the Down edge representation to 1
               obj.g.setDown(obj.y, obj.x, 1);
               obj.g.addNode(obj.y+1, obj.x);
               disp('added node in south');
               obj.moveList = [obj.moveList, 2];
           end
           obj.rotateL90();
           pause(.1);
           dist = obj.getDist();
           fprintf('distance read to east: %f\n', dist);
           % If we can (and havent yet) move south:
           if dist > obj.scanDist && obj.g.checkRight(obj.y, obj.x) == 0
               % set the east node in matrix to 1, and
               % set the Right edge representation to 1
               obj.g.setRight(obj.y, obj.x, 1);
               obj.g.addNode(obj.y, obj.x+1);
               disp('added node to east');
               obj.moveList = [obj.moveList, 3];
           end
           % mark the status of the node to 1 (scanned)
           obj.g.statusArray(obj.y, obj.x) = 1;
           % face north again
           obj.rotateL90();
           pause(.1);
       end
       function obj = rescan(obj)
           % currNode has been scaned already, but there are more paths to
           % explore. figure out what neighbors of currNode havent been
           % scanned yet
           obj.moveList = [];
           if obj.g.checkNode(obj.y-1, obj.x) ~= 0 && obj.g.statusArray(obj.y-1, obj.x) == 0
               obj.moveList = [obj.moveList, 0];
           end
           if obj.g.checkNode(obj.y, obj.x-1) ~= 0 && obj.g.statusArray(obj.y, obj.x-1) == 0
               obj.moveList = [obj.moveList, 1];
           end
           if obj.g.checkNode(obj.y+1, obj.x) ~= 0 && obj.g.statusArray(obj.y+1, obj.x) == 0
               obj.moveList = [obj.moveList, 2];
           end
           if obj.g.checkNode(obj.y, obj.x+1) ~= 0 && obj.g.statusArray(obj.y, obj.x+1) == 0
               obj.moveList = [obj.moveList, 3];
           end
       end
       % when we hit a dead end or have explored all paths from the
       % node, go back (in Depth first search manner)
       % go_back will "flip" the last direction (in moveStack)
       % so we can return to the previous node
       function dir = go_back(lastdir)
           if lastdir == 2
               dir = 0;
           elseif lastdir == 3
               dir = 1;
           elseif lastdir == 0
               dir = 2;
           elseif lastdir == 1
               dir = 3;
           end
       end
       % out of the available moves, randomly decider
       % which one to take and updates the status of the node
       function dir = decide_to_move(obj)
           % If we can make a move to explore a new path
           if ~isempty(obj.moveList)
               % if we can only make 1 move
               if length(obj.moveList) == 1
                   % mark the status of the node to 2, explored
                   obj.g.statusArray(obj.y, obj.x) = 2;
               end
           % randomly decide which movement to take and push to stack
           idx = randi([1, length(obj.moveList)], 1);
           dir = obj.moveList(idx);
           obj.moveStack = [obj.moveStack, dir];
           % if theres no moves, were either at a dead end
           % or we've explored all paths for this node.
           % thus, go back to the previous node from stack.
           else
               obj.g.statusArray(obj.x, obj.y) = 2;
               lastdir = obj.moveStack(end);
               disp(lastdir);
               dir = obj.go_back(lastdir);
           end
       end
       % rotate, move, update the current (y, x), and perform a colorcheck
       function obj = move(obj, dir)
           switch dir
               case 0 % move north
                   obj.orient(0);
                   obj.y = obj.y-1;
                   disp('moving north');
               case 1 % move west
                   obj.orient(1);
                   obj.x = obj.x-1;
                   disp('moving west');
               case 2 % move south
                   obj.orient(2);
                   disp('moving south');
                   obj.y = obj.y+1;
               case 3 % move east
                   obj.orient(3);
                   disp('moving east');
                   obj.x = obj.x+1;
           end
           redSeen = 0;
           for i = 0:6 % move and check if it detects red 5 times
               obj.brick.MoveMotor('AB', -80);
               pause(obj.movePause);
               color = obj.brick.ColorCode(1);
               if color == 5 && redSeen == 0 % if we see red once, stop
                   obj.brick.StopMotor('AB', 'Brake');
                   pause(2);
                   obj.brick.MoveMotor('AB',-80);
                   redSeen = 1;
               end
           end
           obj.brick.StopMotor('AB', 'Brake');
           % handling the origin/pickup/dropoff logic based on color
           color = obj.brick.ColorCode(1);
           switch color
               case 2 % blue = pickup
                   obj.pickupKnown = 1;
                   obj.g.setColor(obj.y, obj.x, 2);
                   % if we havent picked up the person, pick them up
                   if obj.pickedUp == 0
                       disp('move back to node before quitting remote');
                       obj.remote();
                       obj.pickedUp = 1;
                   end
               case 4 % yellow = dropoff
                   obj.dropoffKnown = 1;
                   obj.g.setColor(obj.y, obj.x, 4);
                   % if we picked up person and didnt drop them off
                   if obj.pickedUp == 1 && obj.droppedOff == 0
                       disp('move back to node before quitting remote');
                       obj.remote(); % drop them off
                       obj.droppedOff = 1;
                   end
               case 3 % green = origin
                   if obj.droppedOff == 1
                       obj.allDone = 1;
                   end
           end
       end
       % uses MazeGraph's pathTo to find a list of
       % moves to take to get to the desired node with
       % that color
       function obj = go_to_color(obj, color)
           path = obj.g.pathTo(obj.y, obj.x, color);
           l = len(path) + 1;
           for i = 1:l         % execute the moves
               move = path(i);
               obj.move(move);
           end   
       end
       function obj = run(obj)
           obj.x = 1;
           obj.y = 1;
           while obj.droppedOff == 0
               % explore the maze if we havent found dropoff and pickup
               % yet
               if ~(obj.dropoffKnown == 1 && obj.pickedUp == 1)
                   % make the robot face north so that scan() works
                   % *could have been optimized to save time*
                   obj.orient(0);
                   % if we havent scanned yet, scan
                   if obj.g.statusArray(obj.y, obj.x) == 0
                       obj.scan();
                   else
                       obj.rescan();
                   end % else rescan using what we know already
                       % (we came back to this node if we hit a dead end)
                   % decide to move based on moveList from scan()/rescan()
                   dir = obj.decide_to_move();
                   % move there and repeat
                   obj.move(dir);
               % if we have picked up the passenger and know the dropoff,
               % find the path to the dropoff
               else
                   obj.go_to_color(4);
               end
           end
           % we dropped it off. Now we must return to origin:
           obj.go_to_color(1);
           disp(obj.allDone);
       end
       % Remote control with slow settings for precise movement
       function obj = remote(obj)
           global key
           InitKeyboard() ;
          
           while 1
               pause (0.1);
               switch key
                   case 'k'
                       obj.brick.MoveMotor('C', -20);
                   case 'l'
                       obj.brick.MoveMotor('C', 20);
		            case 'w'
			            obj.brick.MoveMotor('A', -20);
                       obj.brick.MoveMotor('B', -20);
                   case 's'
                       obj.brick.MoveMotor('A', 20);
                       obj.brick.MoveMotor('B', 20);
                   case 'a'
			            obj.brick.MoveMotor('A', 20);
                       obj.brick.MoveMotor('B', -20);
                   case 'd'
			            obj.brick.MoveMotor('A', -20);
                       obj.brick.MoveMotor('B', 20);
                   case 'x'
                       obj.brick.StopMotor('A', 'Brake');
                       obj.brick.StopMotor('B', 'Brake');
                       obj.brick.StopMotor('C', 'Brake');
                   case 'g'
                       break;
               end
           end
           CloseKeyboard();
       end
       % remote control for sumo ring (max movement)
       function obj = remote2(obj)
           global key
           InitKeyboard() ;
          
           while 1
               pause (0.1);
               switch key
                   case 'k'
                       obj.brick.MoveMotor('C', -20);
                   case 'l'
                       obj.brick.MoveMotor('C', 20);
		            case 'w'
			            obj.brick.MoveMotor('A', -100);
                       obj.brick.MoveMotor('B', -100);
                   case 's'
                       obj.brick.MoveMotor('A', 100);
                       obj.brick.MoveMotor('B', 100);
                   case 'a'
			            obj.brick.MoveMotor('A', 60);
                       obj.brick.MoveMotor('B', -60);
                   case 'd'
			            obj.brick.MoveMotor('A', -60);
                       obj.brick.MoveMotor('B', 60);
                   case 'x'
                       obj.brick.StopMotor('A', 'Brake');
                       obj.brick.StopMotor('B', 'Brake');
                       obj.brick.StopMotor('C', 'Brake');
                   case 'g'
                       break;
               end
           end
           CloseKeyboard();
       end
   end
end

