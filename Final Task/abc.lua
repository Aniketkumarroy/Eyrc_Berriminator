function sysCall_init()
    -- do some initialization here
    fl = sim.getObjectHandle('rollingJoint_fl')
    fr = sim.getObjectHandle('rollingJoint_fr')
    rl = sim.getObjectHandle('rollingJoint_rl')
    rr = sim.getObjectHandle('rollingJoint_rr')
    bot = sim.getObjectHandle('BM_Bot')
    --- joints of the arm ---
    joint1 = sim.getObjectHandle('robotic_arm_rj_r1')
    joint2 = sim.getObjectHandle('robotic_arm_pj_12')
    joint3 = sim.getObjectHandle('robotic_arm_pj_23')
    -------------------------
    gripper = sim.getObjectHandle('RG2_leftJoint0')-- gripper joint for knowing whether the gripper is opening or closing or not
    vision_sensor = sim.getObjectHandle('vision_sensor_2')
    local simBase=sim.getObjectHandle('robotic_arm')
    local simTip=sim.getObjectHandle('tip')
    simTarget=sim.getObjectHandle('target')
    --- initially setting 'combined_joint_position' signal to 0%0%0%0 ---
    sim.setStringSignal('combined_joint_position','0%0%0%0')
    -- create an IK environment:
    ikEnv=simIK.createEnvironment()
    -- create an IK group: 
    ikGroup_undamped=simIK.createIkGroup(ikEnv)
    -- set its resolution method to undamped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_undamped,simIK.method_pseudo_inverse,0,6)
    -- create an IK element based on the scene content: 
    simIK.addIkElementFromScene(ikEnv,ikGroup_undamped,simBase,simTip,simTarget,simIK.constraint_position)
    -- create another IK group: 
    ikGroup_damped=simIK.createIkGroup(ikEnv)
    -- set its resolution method to damped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,1,99)
    -- create an IK element based on the scene content: 
    simIK.addIkElementFromScene(ikEnv,ikGroup_damped,simBase,simTip,simTarget,simIK.constraint_position)
    --- scaling of angle turned by joint to distance it moved; s[1] for forward/backward, s[2] for sidewaays, s[3] for diagonal, s[4] for rotation ---
    s = {10.630284245798,11.1,21.778317165359,13/(math.pi/2)}
    --- initial state ---
    X, Y, Alpha = 4,4,0
    --- PID parameters ---
    errsum = {0,0,0,0}
    lasterr = {0,0,0,0}
    lasttime = 0
    kp,ki,kd = 0, 0, 0
    ----------------------
    flag = 0
    p = {} -- to stores the points to be traversed or angle to rotate
    command = {} -- store the command of whether to rotate or translate
end

function sysCall_actuation()
    -- put your actuation code here
    if flag == 1 then --flag =1 start reading target points(p)
        sim.setIntegerSignal('N',0)-- wait Python, we are navigating
        ---getting initial joints positions---
        local joints = sim.getStringSignal('combined_joint_position')
        local t = split(joints,'%%') -- fin initial joint positions
        j1,j2,j3,j4 = tonumber(t[1]),tonumber(t[2]),tonumber(t[3]),tonumber(t[4])
        --------------------------------------
        if command[1] == 1 then--for translation
            x_target, y_target = p[1], p[2]
            x_initial,y_initial = X,Y
            flag = 2
            if Alpha ~= 0 then-- if the bot is not alligned with the world
                -- transform the coordinates W.R.T. to bot --
                x_target,y_target = transform(x_target,y_target,Alpha)
                x_initial,y_initial = transform(X,Y,Alpha)
                ---------------------------------------------
                flag = 4
            end
            if (math.abs(x_target-x_initial) >= 0.1) and (math.abs(y_target-y_initial) >= 0.1) then --for diagonal movement
                target,fb,lr,rot = (y_target-y_initial)*s[3],sign(y_target-y_initial),sign(x_target-x_initial),0
            elseif (math.abs(x_target-x_initial) >= 0.1) then-- for sideways movements
                target,fb,lr,rot = (x_target-x_initial)*s[2],0,1,0
            else--for forward/backward movement
                target,fb,lr,rot = (y_target-y_initial)*s[1],1,0,0
            end
            kp ,ki, kd = 8, 0, 0.1 -- PID parameters for translation
        else--for rotation
            m = s[4]
            if (math.abs(math.pi-math.abs(Alpha)) <= 0.01) and (p[1] ~= 0) then 
                Alpha = sign(p[1])*math.pi
            elseif (math.abs(math.pi/2-math.abs(Alpha)) <= 0.1) and math.abs(math.abs(p[1]) - math.pi) <= 0.01 then 
                p[1] = sign(Alpha)*math.pi
            elseif math.abs(math.abs(p[1]-Alpha)- math.pi) <= 0.01 then 
                m = s[4]*1.015 
            end
            target,fb,lr,rot,flag = (p[1]-Alpha)*m,0,0,1
            kp ,ki, kd = 5, 0, 0.1 -- PID parameters for rotation
            flag = 3
        end
    end
    local now = sim.getSimulationTime()-- records current simulation time(to be used by PID in line-132)
    if flag > 0 then--actuating wheels
        --- getting initial joints positions ---
        local joints = sim.getStringSignal('combined_joint_position')
        local t = split(joints,'%%')
        --------------------------------------
        a1,a2,a3,a4 = tonumber(t[1])-j1,tonumber(t[2])-j2,tonumber(t[3])-j3,tonumber(t[4])-j4-- finding net rotation of joints
        --- calculating the errors ---
        if fb*lr > 0 then
            e1,e2,e3,e4 = target-a1,0,target-a3,0
        elseif fb*lr < 0 then
            e1,e2,e3,e4 = 0,target-a2,0,target-a4
        elseif fb == 1 then
            e1,e2,e3,e4 = target-a1,target-a2,target-a3,target-a4
        elseif lr == 1 then
            e1,e2,e3,e4 = target-a1,-target-a2,target-a3,-target-a4
        else
            e1,e2,e3,e4 = -target-a1,-target-a2,target-a3,target-a4
        end
        -------------------------------
        if math.abs(e1) <= 0.001 and math.abs(e2) <= 0.001 and math.abs(e3) <= 0.001 and math.abs(e4) <= 0.001 then -- we have reached the target
            if flag ~= 3 then X,Y = p[1],p[2] else Alpha = p[1] end -- updating X,Y,Alpha
            --- end of one navigation/rotation step ---
            table.remove(p,1)
            table.remove(command,1)
            if flag ~= 3 then table.remove(p,1) end -- because for translation there are 2 entries in p(x and y)
            --- resetting PID internal parameters ---
            for i = 1,4,1 do
                errsum[i] = 0
                lasterr[i] = 0
            end
            -----------------------------------------
            flag = 1 -- setting coppeliasim to receive next navigation target(line- 51)
            if #p == 0 then
                --- end of navigation and rotation --- 
                e1,e2,e3,e4 = 0,0,0,0
                flag = 0
                sim.setIntegerSignal('N',1)--Python! we have completed navigation and rotation!! :D
            end
        end
        e1,e2,e3,e4 = PID({e1,e2,e3,e4},now-lasttime)-- get the PID output
        setBotMovement(1,e1,e2,e3,e4)-- actuate the joints
    end
    if sim.getIntegerSignal('M') == 0 then -- wait Python! we are manipulating the arm
        if simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_undamped,true)==simIK.result_fail then 
            -- the position could not be reached.
            -- try to solve with the damped method:
            simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_damped)
        else
            if ikFailedReportHandle then
                -- We close any report message about IK failure:
                sim.endDialog(ikFailedReportHandle) 
                ikFailedReportHandle=nil
            end
        end
        if math.abs(sim.getJointVelocity(joint1)) <= 0.05 and math.abs(sim.getJointVelocity(joint2)) <= 0.05 and math.abs(sim.getJointVelocity(joint3)) <= 0.05 then
            sim.setIntegerSignal('M',1)-- Python! we have completed arm manipulation :)
        end
    end
    if sim.getIntegerSignal('G') == 0 and math.abs(sim.getJointVelocity(gripper)) <= 0.05 then
        sim.setIntegerSignal('G',1)--Python! we have completed gripper manipulation :D
    end
    -- sending data to python --
    sim.setFloatSignal('orientation',Alpha)
    sim.setFloatSignal('X',X)
    sim.setFloatSignal('Y',Y)
    sim.setIntegerSignal('I',1)-- Python, all scripts are inialized, you can start now :)
    lasttime = now -- storing lasttime to use it in PID in next script calling
end
function PID(E,t)
    out = {}
    for i = 1,4,1 do
        errsum[i] = errsum[i] + E[i]*t
        derr = (E[i] - lasterr[i])/t
        out[i] = kp*E[i] + kd*derr + ki*errsum[i]
        lasterr[i] = E[i]
    end
    return out[1],out[2],out[3],out[4]
end
function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
--Function Name:setBotMovement
--Input:A(amplification coefficient),e1(joint velocity for frontleft),e2(joint velocity for rearleft),e3(joint velocity for rearright),e4(joint velocity for frontrightt)
--Output:None
--Logic: set the target velocity of the joints of BM_Bot
--Example call:setBotMovement(A,e1,e2,e3,e4)
function setBotMovement(A,e1,e2,e3,e4)
    sim.setJointTargetVelocity(fl,A*e1)
    sim.setJointTargetVelocity(rr,A*e3)
    sim.setJointTargetVelocity(rl,A*e2)
    sim.setJointTargetVelocity(fr,A*e4)
end
--Function Name:transform
--Input:a(x coordinate),b(y coordinate),theta(angle by which to turn the coordinates)
--Output:x_transform(transformed coordinate of x),y_transform(transformed coordinate of y)
--Logic: transform the coordinates W.R.T. one frame to another frame oriented by angle 'theta' W.R.T. to first frame
--Example call:x_transform,y_transform = transform(a,b,theta)
function transform(a,b,theta)--for rotation of coordinates
    x_transform = a*math.cos(theta) + b*math.sin(theta)
    y_transform = -a*math.sin(theta) + b*math.cos(theta)
    return x_transform,y_transform
end
--Function Name:sign
--Input:number(the number whose sign is to be checked)
--Output:x_transform(transformed coordinate of x),y_transform(transformed coordinate of y)
--Logic: give the sign of the input number as 1 or -1
--Example call:sgn = sign(number)
function sign(number)
    if number >= 0 then return 1 else return -1 end
end
--Function Name:split
--Input:inputstr(the input string), sep(the pattern)
--Output:t(table of all subtrings)
--Logic: split a string(inputstr) in substrings based on the repitation of a certain pattern(sep)
--Example call:t = split(inputstr, sep)
function split(inputstr, sep)
    if sep == nil then
        sep = "%s"
    end
    local t={}
    for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
        table.insert(t, str)
    end
    return t
end
--Function Name:Parameter
--Input:inInts(table of ints sent by python),inFloats(table of floats sent by python),inStrings(table of strings sent by python),inBuffer(Buffer sent by python)
--Output:inInts(table of ints sent to python),inFloats(table of floats sent to python),inStrings(table of strings sent to python),inBuffer(Buffer sent to python)
--Logic: for communicating with python, receiving table of points needs to traversed, angles need to be rotated, need for arm manipulation, setting target dummy position, commands
-- NOTE:below example is for calling the function in python
--Example call:return_code,outints,oufloats,outstring,outbuffer = sim.simxCallScriptFunction(client_id,object,sim.sim_scripttype_childscript,function,inInts,inFloats,command,emptybuff,sim.simx_opmode_blocking)
function Parameter(inInts, inFloats, inStrings, inBuffer)--function to receive commands from python
    if inStrings[1] == 'N' then--Navigation
        p = inFloats-- table of points and rotation angles
        command = inInts-- command
        flag = 1 -- start of reading targets points(p)(line-51)
    else
        if inStrings[1]== 'vs' then
            sim.setObjectPosition(simTarget,vision_sensor,inFloats)
        else
            sim.setObjectPosition(simTarget,bot,inFloats)
        end
        sim.setIntegerSignal('M',0)-- wait Python! we are doing arm manipulation (:|)
    end
    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''
    return inInts,inFloats,inStrings,inBuffer
end