classdef MazeGraph < handle
   %{
   Class to manage matrix representations of maze properties
   such as the nodes discovered, their color, whether they
   have been scanned yet and if all paths from it have been
   explored, and a matrix for pathTo function.
   Includes
   %}
   properties
       sX % for pathTo
       sY % for pathTo
       nodeArray % is a node present/discovered?
       colorArray % what color is the node
       % colors: 2 = Blue, 3 = Green, 4 = Yellow, 5 = Red
       searchArray % stores status for search
       % search: 0 = not scanned ,1 = scanned, 2 = all paths explored
       statusArray % stores status for pathTo
       % status: 0 = not visited, 1 = visited, 2 = all paths explored
       upDownArray % up/down mask of edges for nodeArray
       leftRightArray % left/right mask of edges for nodeArray
       r % # of rows for maze representation
       c % # of columns for maze representation
   end
   methods
       function obj = MazeGraph(r, c, y, x)
           % r = num cols of maze
           % c = num rows of maze
           % y = starting y in maze
           % x = starting x in maze
           if nargin == 4
               obj.nodeArray = zeros([r, c]);
               obj.nodeArray(y, x) = 1;
               obj.colorArray = zeros([r, c]);
               obj.colorArray(y, x) = 3;
               obj.statusArray = zeros([r, c]);
               obj.searchArray = zeros([r, c]);
               % upDown array has shape r-1, c
               % represents if undirected up/down edge
               % between two nodes exists
               obj.upDownArray = zeros([r-1 c]);
               % leftRight array has shape r c-1
               % represents if unidrected left/right edge
               % between two nodes exists
               obj.leftRightArray = zeros([r c-1]);
               obj.r = r;
               obj.c = c;
           end
       end
      
       function obj = displayLR(obj)
           disp(obj.leftRightArray);
       end
       function obj = displayUD(obj)
           disp(obj.upDownArray);
       end
       % add a node at the specified y, x (set to 1)
       % if in bounds
       function obj = addNode(obj, y, x)
           if x <= obj.c && y <= obj.r && x > 0 && y > 0
               obj.nodeArray(y, x) = 1;
           end
       end
       % return if the node exists or not
       % and if in bounds (-1 if not)
       function ret = checkNode(obj, y, x)
           if x <= obj.c && y <= obj.r && x > 0 && y > 0
               ret = obj.nodeArray(y, x);
           else
               ret = -1;
           end
       end
       % does an edge to the norht node exist?
       function ret = checkUp(obj, y, x)
           %ret: -1 -> invalid
           %     0  -> open
           %     1  -> exists already
           if y - 1 > 0 && y <= obj.r
               ret = obj.upDownArray(y-1, x);
           else
               ret = -1;
           end
       end
       % does an edge to the south node exist?
       function ret = checkDown(obj, y, x)
           if y < obj.r && y > 0
               ret = obj.upDownArray(y, x);
           else
               ret = -1;
           end
       end
       % does an edge to the west node exist?
       function ret = checkLeft(obj, y, x)
           if x - 1 > 0 && x <= obj.c
               ret = obj.leftRightArray(y, x-1);
           else
               ret = -1;
           end
       end
       % does an edge to the east node exist?
       function ret = checkRight(obj, y, x)
           if x < obj.c && x > 0
               ret = obj.leftRightArray(y, x);
           else
               ret = -1;
           end
       end
       % set the north edge to to s
       function obj = setUp(obj, y, x, s)
           if y - 1 > 0 && y <= obj.r
               obj.upDownArray(y-1, x) = s;
           else
               disp('out of bounds');
           end
       end
       % set the south edge to s
       function obj = setDown(obj, y, x, s)
           if y < obj.r && y > 0
               obj.upDownArray(y, x) = s;
           else
               disp('out of bounds');
           end
       end
       % set the west edge to s
       function obj = setLeft(obj, y, x, s)
           if x - 1 > 0 && x <= obj.c
               obj.leftRightArray(y, x-1) = s;
           else
               disp('out of bounds');
           end
       end
       % set the east edge to s
       function obj = setRight(obj, y, x, s)
           if x < obj.c && x > 0
               obj.leftRightArray(y, x) = s;
           else
               disp('out of bounds');
           end
       end
       % check the color of the node
       function ret = checkColor(obj, y, x)
           if x <= obj.c && y <= obj.r && x > 0 && y > 0
               ret = obj.colorArray(y, x);
           end
       end
       % set the color of the node
       function obj = setColor(obj, y, x, s)
           if x <= obj.c && y <= obj.r && x > 0 && y > 0
               obj.colorArray(y, x) = s;
           end
       end
       % colors: 2 = Blue, 3 = Green, 4 = Yellow, 5 = Red
       % DFS-like search of known maze to find a path from the
       % node at x, y to the to the node of the specified color, s.
       % Return a list of moves to get there
       function path = pathTo(obj, y, x, s)
           obj.sY = y; % manage the position while searching
           obj.sX = x; % maange the position while searching
           found = 0;  % loop condition
           path = [];  % manage the list of moves
           while found == 0
               moveList = [];  % manage the possible moves from sY, sX
               obj.searchArray(obj.sY, obj.sX) = 1; % scanning current node
               q = obj.checkColor(obj.sY, obj.sX); % check its color first
               if q == s
                   found = 1;
               else % if its not the desired color, find what moves can be
                    % made
                   ret1 = obj.checkUp(obj.sY, obj.sX);
                   % if we can (and have yet to) search up, add that to
                   % the list
                   if ret1 > 0 % it exists
                       ret2 = obj.searchArray(obj.sY-1, obj.sX);
                       if ret2 < 1 % and we have yet to explore it
                           moveList = [moveList, 0];
                       end
                   end
                   % if we can (and have yet to) search left, add that to
                   % the list
                   ret1 = obj.checkLeft(obj.sY, obj.sX);
                   if ret1 > 0
                       ret2 = obj.searchArray(obj.sY, obj.sX-1);
                       if ret2 < 1
                           moveList = [moveList, 1];
                       end
                   end
                   % if we can (and have yet to) search down, add that to
                   % the list
                   ret1 = obj.checkDown(obj.sY, obj.sX);
                   if ret1 > 0
                       ret2 = obj.searchArray(obj.sY+1, obj.sX);
                       if ret2 < 1
                           moveList = [moveList, 2];
                       end
                   end
                   % if we can (and have yet to) search right, add that to
                   % the list
                   ret1 = obj.checkRight(obj.sY, obj.sX);
                   if ret1 > 0
                       ret2 = obj.searchArray(obj.sY, obj.sX+1);
                       if ret2 < 1
                           moveList = [moveList, 3];
                       end
                   end
                   % if we can make moves
                   if ~isempty(moveList)
                       % if we can only make 1, mark status as explored
                       if length(moveList) == 1
                           obj.searchArray(obj.sY, obj.sX) = 2;
                       end
                       % randomly decide how to move
                       idx = randi([1,length(moveList)], 1);
                       dir = moveList(idx);
                       path = [path, dir];
                       % update sY or sX based on decision
                       switch dir
                           case 0
                               obj.sY = obj.sY-1;
                           case 1
                               obj.sX = obj.sX-1;
                           case 2
                               obj.sY = obj.sY+1;
                           case 3
                               obj.sX = obj.sX+1;
                       end
                   else % if we have no moves to make
                       % go back to last sY, sX
                       obj.searchArray(obj.sY, obj.sX) = 2;
                       lastDir = path(end);
                       path(end) = [];
                       switch lastDir
                           case 0
                               obj.sY = obj.sY+1;
                           case 1
                               obj.sX = obj.sX+1;
                           case 2
                               obj.sY = obj.sY-1;
                           case 3
                               obj.sX = obj.sX-1;
                       end
                   end
               end
               disp(obj.searchArray);
           end
       end
   end
end