-- The University of Sheffield
-- ACS6121 Robotics and Autonomous Systems Spring 2018/19
-- V-rep Simulation Assignment 
-- R. No. : 180123717 
-- Name: Paulo Roberto Loma Marconi
---------------------------------------------------------------------------------------------------------
sim.setThreadAutomaticSwitch(false) -- manually switch the thread so we can control the sample period

-- init randomseed
math.randomseed(os.time()) 
math.random(); math.random(); math.random()

-- global constants
T=200 -- sample period [ms]
pi=math.pi

-- Bubble Rebound algorithm constants |\label{line:ReboundConstants}|  
N=6; alpha0=pi/N;

alphaR=0 -- [rad]
omega=0 -- [rad/s]

-- e-puck constants |\label{line:ePuckConstants}|   
-- http://www.e-puck.org/index.php?option=com_content&view=article&id=7&Itemid=9 
-- http://www.gctronic.com/e-puck_spec.php 
maxWheelVel=6.24 -- Max angular wheel speed 6.24[rad/s]
maxVx=0.127 -- Max robot linear velocity, 0.127[m/s]=12.7[cm/s]
L=0.051 -- 5.1 cm, distance between the wheels
D=0.041 -- 4.1 cm, wheel diameter
R=D/2 -- wheel radius

timeSimul=60 -- simulation time threshold [s]

-- Functions: -------------------------------------------------------------------------------------------
-- Color Blob detection
function colorDetect(idx,blobPosX,blobPosY)
    local blobCol=sim.getVisionSensorImage(ePuckCam,resu[1]*blobPosX[idx],resu[2]*blobPosY[idx],1,1)    
    if (blobCol[1]>blobCol[2])and(blobCol[1]>blobCol[3]) then color='R' end
    if (blobCol[2]>blobCol[1])and(blobCol[2]>blobCol[3]) then color='G' end
    if (blobCol[3]>blobCol[1])and(blobCol[3]>blobCol[2]) then color='B' end
    return color
end

-- Biggest Blob
function bigBlob(blobSize)
    local maxVal,idx=-math.huge
    for k,v in pairs(blobSize) do
        if v>maxVal then
            maxVal,idx=v,k
        end
    end
    return idx
end

---------------------------------------------------------------------------------------------------------
-- This is the Epuck principal control script. It is threaded
threadFunction=function()
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do       
    t=sim.getSimulationTime()

-- Image Processing Part ================================================================================
    sim.handleVisionSensor(ePuckCam) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
    result,t0,t1=sim.readVisionSensor(ePuckCam) -- Here we read the image processing camera!    
    resu=sim.getVisionSensorResolution(ePuckCam) -- Color blob detection init

-- The e-puck robot has Blob Detection filter. The code provided below get useful information
-- regarding blobs detected, such as amount, size, position, etc.
    
    -- t1[1]=blob count, t1[2]=dataSizePerBlob=value count per blob=vCnt, 
    -- t1[3]=blob1 size, t1[4]=blob1 orientation, 
    -- t1[5]=blob1 position x, t[6]=blob1 position y,
    -- t[7]=blob1 width, t[8]=blob1 height, ..., (3+vCnt+0) blob2 size,
    -- (3+vCnt+1) blob2 orientation, etc.

    pO={0.5,0} --[x0,y0] Relative Robot position in the 2D image
    blobSize={0}; blobOrientation={0}; 
    blobPos={0}; blobPosX={0}; blobPosY={0}
    blobBoxDimensions={0}; 
    blobColor={0}; 
    blobRedSize={0}; blobRedPosX={0}; blobRedPosY={0};
    blobGreenSize={0}; blobGreenPosX={0}; blobGreenPosY={0};
    ePuckOrientation={0}; ePuckSize={0}; ePuckPos={0}; ePuckPosX={0}; ePuckPosY={0};

    if (t1) then -- (if Detection is successful) in t1 we should have the blob information if the camera was set-up correctly  
        blobCount=t1[1]
        dataSizePerBlob=t1[2]
        lowestYofDetection=100
        -- Now we go through all blobs:
        for i=1,blobCount,1 do
            blobSize[i]=t1[2+(i-1)*dataSizePerBlob+1]
            blobOrientation[i]=t1[2+(i-1)*dataSizePerBlob+2]
            blobPos[i]={t1[2+(i-1)*dataSizePerBlob+3],t1[2+(i-1)*dataSizePerBlob+4]} --[pos x,pos y]
            blobPosX[i]=t1[2+(i-1)*dataSizePerBlob+3]
            blobPosY[i]=t1[2+(i-1)*dataSizePerBlob+4]
            blobBoxDimensions[i]={t1[2+(i-1)*dataSizePerBlob+5],t1[2+(i-1)*dataSizePerBlob+6]} -- [w,h]                        
            -- Color detection of all blobs and group them by two vectors (Green and Red)  |\label{line:colours}|
            blobColor[i]=colorDetect(i,blobPosX,blobPosY) 
            if (blobColor[i]=='R') then 
                blobRedSize[i]=blobSize[i]; blobRedPosX[i]=blobPosX[i]; blobRedPosY[i]=blobPosY[i];  
            end
            if (blobColor[i]=='G') then 
                blobGreenSize[i]=blobSize[i]; blobGreenPosX[i]=blobPosX[i]; blobGreenPosY[i]=blobPosY[i];  
            end
            -- Detect the orientation, size and position of the detected ePucks    |\label{line:wheels}|
            if (blobOrientation[i]~=-0) then
                ePuckOrientation[i]=blobOrientation[i];
                ePuckSize[i]=blobSize[i]; ePuckPos[i]=blobPos[i];
                ePuckPosX[i]=blobPosX[i]; ePuckPosY[i]=blobPosY[i];
                flagEPuck=1;
            end
        end
    end
    
-- Proximity sensor readings ===========================================================================    
    s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
    noDetectionDistance=0.05*s
    proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
    for i=1,8,1 do
        res,dist=sim.readProximitySensor(proxSens[i])   
        if (res>0) and (dist<noDetectionDistance) then
            proxSensDist[i]=dist                  
        end
    end

-- Controller Algorithm =================================================================================
    
    -- Behaviour state: ---------------------------------------------------------------------------------|\label{line:behaviour}|  
    if (t<=timeSimul) then behaviour='avoider'
    else behaviour='follower' end

    -- Define the weight vector |\label{line:vector}|
    if (behaviour=='avoider') then
        alpha={-3*alpha0,-2*alpha0,-1*alpha0,1*alpha0,2*alpha0,3*alpha0} -- avoider weight vector
    elseif (behaviour=='follower') then 
        alpha={3*alpha0,2*alpha0,1*alpha0,-1*alpha0,-2*alpha0,-3*alpha0} -- follower weight vector
    end 
    
	-- Rebound avoider/follower algorithm ---------------------------------------------------------------|\label{line:rebound}|  
    -- Calculate the angle of attack alphaR 
    sum_alphaD=0; sumD=0; 
    for j=1,N,1 do          
        sum_alphaD=sum_alphaD+alpha[j]*proxSensDist[j]
    end        
    for j=1,N,1 do
        sumD=sumD+proxSensDist[j]
    end    
    alphaR=sum_alphaD/sumD            

    -- Foraging State: ----------------------------------------------------------------------------------|\label{line:foraging}|  
	-- Search blobb/ePuck algorithm 
    -- Find the biggest green Blob index using the blobGreenSize vector data   
    idx=bigBlob(blobGreenSize)

    -- Angle to the closest green Blob given by the biggest blob idx 
    alphaC=math.atan((blobGreenPosX[idx]-pO[1])/(blobGreenPosY[idx]-pO[2]))

    -- Random State: Makes a random movement when there is no Blob detection |\label{line:random1}|
    if (math.deg(alphaC)==-90) or (math.deg(alphaC)==90) then
        alphaC=2*alphaC*math.random(-1,1)   
    end

    -- Agregation State for 60<t<=120: ------------------------------------------------------------------
    -- Find the biggest ePuck wheel	
	idxEPuck=bigBlob(ePuckSize)  
	-- Angle to the biggest ePuck wheel
    alphaEPuck=math.atan((ePuckPosX[idxEPuck]-pO[1])/(ePuckPosY[idxEPuck]-pO[2]))
    
-- Ouput ================================================================================================    

    -- Vx for avoider/follower --------------------------------------------------------------------------
    threshold=0.015 -- threshold detection
    Vx=maxVx -- go straight
    
    if (behaviour=='avoider') then       
        if (proxSensDist[2]<threshold) or (proxSensDist[3]<threshold) or (proxSensDist[4]<threshold) or (proxSensDist[5]<threshold)  then      
            Vx=0; -- stop robot
            -- Corrected angle due the symmetrical obstacle in front of the robot, only applicable in the avoider
            if alphaR==0 then alphaR=pi*math.random(-1,1) end 
        end    
    end
    
    if (behaviour=='follower') then 
        Vx=maxVx
        if (proxSensDist[2]<threshold) or (proxSensDist[3]<threshold) or (proxSensDist[4]<threshold) or (proxSensDist[5]<threshold) then 
            Vx=0; -- stop robot
        end
    end
     
    -- Obstacle Detection/noDetection flag --------------------------------------------------------------
    if (proxSensDist[1]==0.05)and(proxSensDist[2]==0.05)and(proxSensDist[3]==0.05)and(proxSensDist[4]==0.05)and(proxSensDist[5]==0.05)and(proxSensDist[6]==0.05) then
        flag='noObsDetection'
    else
        flag='ObsDetection'
    end

    -- Output omega [rad/s]=instantaneous robot angular velocity. T[ms]/1000[ms]=t[s] -------------------|\label{line:omega}|
    if (t<=timeSimul) then -- avoider+ObsDetection/noObsDetection+alphaR+alphaC  
        if (flag=='ObsDetection') then     
            omega=alphaR/(T/1000); flg='Rebound';               
        elseif (flag=='noObsDetection') then           
            omega=alphaC/(T/1000); flg='Camera';    
        end
    else                   -- follower +alphaEPuck
        -- Random state: Random movement when there is no ePuck detection |\label{line:random2}|
        if (math.deg(alphaEPuck)==-90) or (math.deg(alphaEPuck)==90) then
            alphaEPuck=2*alphaEPuck*math.random(-1,1)   
        end
        if (flagEPuck~=1) then
            alphaEPuck=0;
        end            
        omega=alphaEPuck/(T/1000); flg='ePuck';        
    end

    -- Angular velocities of the wheels using the Unicycle model, vr and vl  ----------------------------|\label{line:unicycle}|
    velLeft=(2*Vx+omega*L)/(2*R); -- rad/s 
    velRight=(2*Vx-omega*L)/(2*R); -- rad/s

    -- Wheel velocity constraints -----------------------------------------------------------------------
    if (velLeft>maxWheelVel) then velLeft=maxWheelVel
    elseif (velLeft<-maxWheelVel) then velLeft=-maxWheelVel end
    if (velRight>maxWheelVel) then velRight=maxWheelVel
    elseif (velRight<-maxWheelVel) then velRight=-maxWheelVel end
  
    -- Right/Left motor output  -------------------------------------------------------------------------
    sim.setJointTargetVelocity(leftMotor,velLeft)  
    sim.setJointTargetVelocity(rightMotor,velRight)

	print('time',t,'behaviour',behaviour,'flg',flg,'Vx',Vx,'omega',omega,'velLeft',velLeft,'velRight',velRight)
	
    sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
                       -- we switch the thread now!   


    end -- end while
end --  end thread function

---------------------------------------------------------------------------------------------------------
-- These are handles, you do not need to change here. (If you need e.g. bluetooth, you can add it here)

sim.setThreadSwitchTiming(T) -- We will manually switch in the main loop (200)
bodyElements=sim.getObjectHandle('ePuck_bodyElements')
leftMotor=sim.getObjectHandle('ePuck_leftJoint')
rightMotor=sim.getObjectHandle('ePuck_rightJoint')
ePuck=sim.getObjectHandle('ePuck')
ePuckCam=sim.getObjectHandle('ePuck_camera')
ePuckBase=sim.getObjectHandle('ePuck_base')
ledLight=sim.getObjectHandle('ePuck_ledLight')

proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
end


res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    sim.addStatusbarMessage('Lua runtime error: '..err)
end

