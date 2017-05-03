-- This is the Epuck principal control script. It is threaded


-- FUNCTION MOVE
-- Moving the robot
-- distance : distance (meters)
-- velLeft : angular speed left wheel
-- velRight : angular speed rigth wheel
move = function(distance, velLeft, velRight)
    simSetJointTargetVelocity(leftMotor, velLeft)
    simSetJointTargetVelocity(rightMotor, velRight)
    
    local i = 0
    local distanceParcourue = 0
    local p = simGetObjectPosition(ePuckBase, -1)
    local p1 = simGetObjectPosition(ePuckBase, -1)
       
    -- time-out : variable i
    while (i < 1000 and distanceParcourue < distance) do
        p1 = simGetObjectPosition(ePuckBase, -1)
        distanceParcourue = distanceMes(p[1], p[2], p1[1], p1[2])
        --simAddStatusbarMessage('parcourue :' .. distanceParcourue .. ' - Attendue :' .. distance)
        i = i + 1
        simSwitchThread() 
    end
   
    simSetJointTargetVelocity(leftMotor,0)
    simSetJointTargetVelocity(rightMotor,0)
end





-- FUNCTION TURNLEFT
-- Make a left turn (90°)
-- vel : angular speed of wheels
turnLeft=function(vel)
   local rotAmount=-math.pi/2
   local sign=rotAmount/math.abs(rotAmount)
   simSetJointTargetVelocity(leftMotor,vel*sign*0.5)
   simSetJointTargetVelocity(rightMotor,-vel*sign*0.5)
   local previousAngle=simGetObjectOrientation(ePuckBase,-1)[3]
   local rot=0
   local i = 0

   while (i < 1000) do --and (math.abs(rot) < math.pi/2)
      local angle=simGetObjectOrientation(ePuckBase,-1)[3]
      local da=angle-previousAngle
      if da>=0 then
         da=math.mod(da+math.pi,2*math.pi)-math.pi
      else
         da=math.mod(da-math.pi,2*math.pi)+math.pi
      end
      rot=rot+da
      previousAngle=angle
      if math.abs(rot)>math.pi/2 then
         break
      end
      simSwitchThread()
   end
   simSetJointTargetVelocity(leftMotor,0)
   simSetJointTargetVelocity(rightMotor,0)
end



-- FUNCTION TURNRIGHT
-- Make a right turn (90°)
-- vel : angular speed of wheels
turnRight=function(vel)
   local rotAmount=math.pi/2
   local sign=rotAmount/math.abs(rotAmount)
   simSetJointTargetVelocity(leftMotor,vel*sign*0.5)
   simSetJointTargetVelocity(rightMotor,-vel*sign*0.5)
   local previousAngle=simGetObjectOrientation(ePuckBase,-1)[3]
   local rot=0
   local i = 0

   while (i < 1000) do --and (math.abs(rot) < math.pi/2) 
      local angle=simGetObjectOrientation(ePuckBase,-1)[3]
      local da=angle-previousAngle
      if da>=0 then
         da=math.mod(da+math.pi,2*math.pi)-math.pi
      else
         da=math.mod(da-math.pi,2*math.pi)+math.pi
      end
      rot=rot+da
      previousAngle=angle
      if math.abs(rot)>math.pi/2 then
         break
      end
      simSwitchThread()
   end
   simSetJointTargetVelocity(leftMotor,0)
   simSetJointTargetVelocity(rightMotor,0)
end


-- FONCTION UTURN
-- Make a half turn (180°)
-- vel : angular speed of wheels
uTurn=function(vel)
   local rotAmount=math.pi/2
   local sign=rotAmount/math.abs(rotAmount)
   simSetJointTargetVelocity(leftMotor,vel*sign*0.5)
   simSetJointTargetVelocity(rightMotor,-vel*sign*0.5)
   local previousAngle=simGetObjectOrientation(ePuckBase,-1)[3]
   local rot=0
   local i = 0

   while (i < 1000) do --and (math.abs(rot) < math.pi/2) 
      local angle=simGetObjectOrientation(ePuckBase,-1)[3]
      local da=angle-previousAngle
      if da>=0 then
         da=math.mod(da+math.pi,2*math.pi)-math.pi
      else
         da=math.mod(da-math.pi,2*math.pi)+math.pi
      end
      rot=rot+da
      previousAngle=angle
      if math.abs(rot)>math.pi then
         break
      end
      simSwitchThread()
   end
   simSetJointTargetVelocity(leftMotor,0)
   simSetJointTargetVelocity(rightMotor,0)
end




-- FUNCTION DISTANCE
-- Measure the absolute distance between point A(xa, yx) and B(xb, yb)
-- xa : coordinates x of first point (A)
-- ya : coordinates y of first point (A)
-- xb : coordinates x of second point (B)
-- yb : coordinates y of second point (B)
-- return : absolute distance between points
distanceMes = function(xa, ya, xb, yb)
    return math.sqrt( math.pow((xb - xa), 2) + math.pow((yb - ya), 2) )
end


-- FUNCTION SENSORS
-- Save actual state (true/false) of sensors
sensors = function()
    local boolSensors = 0x0
    local sc=simGetObjectSizeFactor(bodyElements)
    local r = 0
    local d = 0
    local i=0

    for i=1,8,1 do
        --r,d = simCheckProximitySensor(proxSens[i], sim_handle_all)
        r,d = simReadProximitySensor(proxSens[i])


        mask = math.pow(2, (i-1))
        notMask = simBoolXor32(mask, 0xFFFFFFFF)

--        if res > 0 and dist < 0.05*s then
        if (r > 0) then  
          boolSensors = simBoolOr32(boolSensors, mask)
            --simAddStatusbarMessage('sens[' .. i .. '] - ' .. r .. ' - ' .. d)            
        else
            boolSensors = simBoolAnd32(boolSensors, notMask)
            --simAddStatusbarMessage('sens[' .. i .. ']') 
        end

        
    end

    return boolSensors
end


-- NOT USED
moveDistance = function(distance, vel, vel2)
    local i = 0
    local distanceParcourue = 0
    local p = simGetObjectPosition(ePuckBase, -1)
    local p1 = simGetObjectPosition(ePuckBase, -1)
    bodyElements=simGetObjectHandle('ePuck_bodyElements')
    s=simGetObjectSizeFactor(bodyElements)

    simSetJointTargetVelocity(leftMotor,vel)
    simSetJointTargetVelocity(rightMotor,vel)

    velLeft=vel
    velRight=vel
    maxVel = vel

    while (i < 1000 and distanceParcourue < distance) do

        noDetectionDistance=0.05*s --0.05
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=simReadProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end
    
        if (proxSensDist[1] < 0.2*noDetectionDistance) then
            velLeft=velLeft + maxVel * 0
            velRight=velRight + maxVel * -0.05 * (1-(proxSensDist[2]/noDetectionDistance))
            --velRight=velRight + maxVel * -0.05
        elseif (proxSensDist[6] < 0.2*noDetectionDistance) then
            velLeft=velLeft + maxVel * -0.05 * (1-(proxSensDist[4]/noDetectionDistance))
            --velLeft=velLeft + maxVel * -0.05
            velRight=velRight + maxVel * 0
        else
            velLeft=vel
            velRight=vel
        end

        p1 = simGetObjectPosition(ePuckBase, -1)
        distanceParcourue = distanceMes(p[1], p[2], p1[1], p1[2])
        i = i + 1
        simSetJointTargetVelocity(leftMotor,velLeft)
        simSetJointTargetVelocity(rightMotor,velRight)
        simSwitchThread() 
    end --while

        simSetJointTargetVelocity(leftMotor,0)
        simSetJointTargetVelocity(rightMotor,0)

end --function


actualizeLEDs=function()
    if (relLedPositions==nil) then
        relLedPositions={{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        simRemoveDrawingObject(drawingObject)
    end
    type=sim_drawing_painttag+sim_drawing_followparentvisibility+sim_drawing_spherepoints+
        sim_drawing_50percenttransparency+sim_drawing_itemcolors+sim_drawing_itemsizes+
        sim_drawing_backfaceculling+sim_drawing_emissioncolor
    drawingObject=simAddDrawingObject(type,0,0,bodyElements,27)
    m=simGetObjectMatrix(ePuckBase,-1)
    itemData={0,0,0,0,0,0,0}
    simSetLightParameters(ledLight,0)
    for i=1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p=simMultiplyVector(m,relLedPositions[i])
            itemData[1]=p[1]
            itemData[2]=p[2]
            itemData[3]=p[3]
            itemData[4]=ledColors[i][1]
            itemData[5]=ledColors[i][2]
            itemData[6]=ledColors[i][3]
            simSetLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j=1,3,1 do
                itemData[7]=j*0.003
                simAddDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

getLightSensors=function()
    data=simReceiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=simUnpackFloats(data)
    end
    return lightSens
end

threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        st=simGetSimulationTime()
        opMode=simGetScriptSimulationParameter(sim_handle_self,'opMode')
        lightSens=getLightSensors()
        s=simGetObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        DetectionDistance = 0.2*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        
    
        for i=1,8,1 do
            res,dist=simReadProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end

        -- ##################################
        -- COMMUNICATION WITH THE JAVA CODE
        cde = simGetIntegerSignal("cde")
        distance = simGetIntegerSignal("distance")
        vel = simGetIntegerSignal("vel")
        --simSetIntegerSignal("return", 0)
        
        bs = tonumber(sensors())
        if(bs ~= bsprec ) then
            --simAddStatusbarMessage('\t\tsens ' .. bs .. ' - prec ' .. bsprec)
            simSetIntegerSignal("sensors", bs)
        end
        bsprec = bs
        
        -- Commands provided by Java code
        if(newCde) then
            newCde = false
            if(cde == 1 and distance > 0 and vel > 0 ) then
                simAddStatusbarMessage('cde move '.. distance/100 .."m at " .. vel)
                --moveDistance(distance/100, vel, vel)
                move(distance/100, vel, vel)
                simSetIntegerSignal("return", 1)            
            elseif(cde == 2) then
                simAddStatusbarMessage('cde turn left at ' .. vel)
                turnLeft(vel)
                simSetIntegerSignal("return", 2)
            elseif(cde == 3) then
                simAddStatusbarMessage('cde turn right at ' .. vel)
                turnRight(vel)
                simSetIntegerSignal("return", 3)
            elseif(cde == 4) then
                simAddStatusbarMessage('cde u-turn at ' .. vel)
                uTurn(vel)
                simSetIntegerSignal("return", 4)
            end
        end

        -- Detection of a new command
        if (cde == 10) then
            newCde = true
            simSetIntegerSignal("return", 0)
        end

        -- ##################################
                
    
        actualizeLEDs()
        simSwitchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

-- Put some initialization code here:
simSetThreadSwitchTiming(200) -- We will manually switch in the main loop
bodyElements=simGetObjectHandle('ePuck_bodyElements')
leftMotor=simGetObjectHandle('ePuck_leftJoint')
rightMotor=simGetObjectHandle('ePuck_rightJoint')
ePuck=simGetObjectHandle('ePuck')
ePuckBase=simGetObjectHandle('ePuck_base')
ledLight=simGetObjectHandle('ePuck_ledLight')
proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=simGetObjectHandle('ePuck_proxSensor'..i)
end
maxVel=120*math.pi/180
ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
p = simGetObjectPosition(ePuckBase, -1)
p1 = simGetObjectPosition(ePuckBase, -1)
boolSensors = 0x0000
cde = 0
newCde = false
bsprec = 0
bs = 0

-- Braitenberg weights for the 4 front prox sensors (avoidance):
braitFrontSens_leftMotor={1,2,-2,-1}
-- Braitenberg weights for the 2 side prox sensors (following):
braitSideSens_leftMotor={-1,0}
-- Braitenberg weights for the 8 sensors (following):
braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

simSetJointTargetVelocity(leftMotor,0)
simSetJointTargetVelocity(rightMotor,0)

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Put some clean-up code here:

for i=1,9,1 do
    ledColors[i]={0,0,0} -- no light
end
actualizeLEDs()

