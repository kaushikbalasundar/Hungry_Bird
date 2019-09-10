function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDrone_outer')
    collection_handles= sim.getCollectionHandle('Obstacles')

    -- Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------
    initial_waypoint_handle = sim.getObjectHandle('initial_waypoint')
    goal_1_handle = sim.getObjectHandle('goal_1')
    goal_2_handle = sim.getObjectHandle('goal_2')
    
    --Hint : Goal handles and other required handles
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------
    t1=simOMPL.createTask('t1')
    t2=simOMPL.createTask('t2')
    t3=simOMPL.createTask('t3')

    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,initial_waypoint_handle,{-3,-2.5,0},{3,2.5,1.5},1)}

    simOMPL.setStateSpace(t1,ss)
    simOMPL.setStateSpace(t2,ss)
    simOMPL.setStateSpace(t3,ss)

    simOMPL.setAlgorithm(t1,simOMPL.Algorithm.RRTConnect)
    simOMPL.setAlgorithm(t2,simOMPL.Algorithm.RRTConnect)
    simOMPL.setAlgorithm(t3,simOMPL.Algorithm.RRTConnect)

    simOMPL.setCollisionPairs(t1,{sim.getObjectHandle('eDrone_outer'),collection_handles})
    simOMPL.setCollisionPairs(t2,{sim.getObjectHandle('eDrone_outer'),collection_handles})
    simOMPL.setCollisionPairs(t3,{sim.getObjectHandle('eDrone_outer'),collection_handles})

    -- no of path points minimum required
    
    compute_path2_flag = false
    compute_path3_flag = false




    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    drone_path_plan_pub=simROS.subscribe('/drone_path_plan_pub', 'std_msgs/Float64', 'drone_path_plan_callback')





    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------


    scale_factor = {-7.5532690593786, -7.5609203149575, 18.229531523408} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required = 70 -- Add no of path points you want from one point to another

end

function drone_path_plan_callback(msg)
    data = msg.data

    if (data == 1) then
        compute_path1_flag = true
        
    elseif (data == 2) then
        compute_path2_flag = true

    else 

        compute_path3_flag = true
    end 
end





function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end



-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end


-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)
    whycon_ground_z_value = 55.60
    scale_factor = {-7.5532690593786, -7.5609203149575, 18.229531523408}
    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x = (path[i]) * scale_factor[1]
        pose.position.y = (path[i+1])* scale_factor[2]
        pose.position.z =  whycon_ground_z_value- (path[i+2]* scale_factor[3])
        sender.poses[math.floor(i/7) + 1] = pose

        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    return sender
end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path

    r,path=simOMPL.compute(task,10,-1,no_of_path_points_required) -- Provide the correct arguments here.. Make sure you check the no of path points it actually computes
    print(r, #path)
    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end


function sysCall_actuation()
    
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    if compute_path1_flag == true then
        -- Getting startpose
        start_pose = getpose(initial_waypoint_handle,-1)
        -- Getting the goalpose
        goal_pose = getpose(goal_1_handle,-1)
        -- Setting start state
        simOMPL.setStartState(t1,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t1,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t1)
        if(status == true) then -- path computed
            compute_path1_flag = false
        end
    end 

    if compute_path2_flag == true then
        -- Getting startpose
        start_pose = getpose(goal_1_handle,-1)
        -- Getting the goalpose
        goal_pose = getpose(goal_2_handle,-1)
        -- Setting start state
        simOMPL.setStartState(t2,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t2,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t2)
        if(status == true) then -- path computed
            compute_path2_flag = false
        end
    end 
    if compute_path3_flag == true then
        -- Getting startpose
        start_pose = getpose(goal_2_handle,-1) 
        -- Getting the goalpose
        goal_pose = getpose(initial_waypoint_handle,-1)
        -- Setting start state
        simOMPL.setStartState(t3,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t3,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t3)
        if(status == true) then -- path computed
            compute_path3_flag = false
        end
    end 

    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end


function sysCall_sensing()

end



function sysCall_cleanup()

end

