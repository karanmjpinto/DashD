function [RL,Result] = Throttle(RL,CD,RD,Result)
% POWERTHROTTLE Summary:
% Torque output of the motor
%% Required accelerations for testing
%TURN=1;
RL.Override='N';
TrackData=RD.Topo;

LookAheadDist = 300;
AcclSectLength = 15;
MinAccel = 0.2;

Decel = 3; %m/s^2

%% Checking Concordance
RL.VRaceAverage = RL.DistanceTotal./RL.Clock;
RL.VLapAverage = (RL.i+1)./RL.LapTime;

%Set TURN, which applied braking if going too quickly
%https://en.wikipedia.org/wiki/Centripetal_force
shouldBrake = false;
TurnR = TrackData.TurnR(RL.i:RL.i+100);
TurnR = TurnR(~isnan(TurnR));
for i = RL.i:RL.i+100
    RadLkAhd = TrackData.TurnR(i);
    if ~isnan(TurnR)
        MaxCornSpeed = sqrt((200*min(RadLkAhd))/CD.M);
        DistToCorn = TrackData.Distance(i) - TrackData.Distance(RL.i);
        if ((RL.VSpeedIn - MaxCornSpeed)/Decel) > (DistToCorn/RL.VSpeedIn)
            shouldBrake = true;
        end
        
        break;
    end
end
if numel(TurnR) > 0
    LatForce = (CD.M*(RL.VSpeedIn^2))/min(TurnR);
    TURN = 200/LatForce;
else
    TURN = 1;
end
if TURN>=1
    TURN=1;
    RL.Notes='';
else
    RL.Notes=char(['Turning too fast, turn: ',num2str(TURN)]);
end


MinCornI = LookAheadDist;
MinCorn = nan;
for i = RL.i:RL.i+LookAheadDist
    if ~isnan(TrackData.TurnR(i))
        MinCorn = TrackData.TurnR(i);
        MinCornI = i;
        break;
    end
end
HeightLkAhd = TrackData.Z(RL.i:MinCornI);

[MaxHeight, MaxHeightI] = max(HeightLkAhd);

HeightDifZ = MaxHeight - TrackData.Z(RL.i);
HeightDifCorn = TrackData.Z(MinCornI) - TrackData.Z(RL.i);

if (HeightDifCorn < 0) && (MinCornI < MaxHeightI) %Going downhill into a corner, before a hill
    MaxCornSpeed = sqrt((200*MinCorn)/CD.M);
    disp(['Max Corn Speed: ',num2str(MaxCornSpeed)])
    if (RL.TargetSpeed > MaxCornSpeed)
        RL.TargetSpeed = MaxCornSpeed;
    else
        RL.TargetSpeed = RD.TargetSpeedConst;
    end
    
elseif (HeightDifZ > 0) %No important corner, but hill needed to climb
    NeedSpeed = sqrt(9.81*2*HeightDifZ);
    if (NeedSpeed > RD.TargetSpeedConst)
        RL.TargetSpeed = NeedSpeed;
    else
        RL.TargetSpeed = RD.TargetSpeedConst;
    end
    
else %Revert to normal
    RL.TargetSpeed = RD.TargetSpeedConst;
end

%Calculate correct action to take
if shouldBrake
    RL.Notes=char([RL.Notes ' BRAKED - Look ahead']);
    RL.VSpeedIn = sqrt(RL.VSpeedIn^2 - (2*Decel*TrackData.d(RL.i)));
    RL.AcclCount = 0;
    
elseif TURN<1
    RL.Notes=char([RL.Notes ' BRAKED']);
    RL.VSpeedIn = TURN.*RL.VSpeedIn;
    RL.AcclCount = 0;
    
elseif RL.AcclCount > 0 %Override overridden
    RL.PercentageRun = (RL.TargetSpeed-RL.VSpeedIn)./RL.TargetSpeed;
    RL.AValReq = ((RL.TargetSpeed-RL.VSpeedIn)./RL.TargetSpeed)*CD.AMax;
    if RL.PercentageRun<MinAccel
        RL.PercentageRun=MinAccel;
    end
    
    if RL.VSpeedIn < RL.TargetSpeed
        %RL.PercentageRun = RL.PercentageRun.*2;
    end
    
    RL.AcclCount = RL.AcclCount - 1;
    
elseif (RL.VSpeedIn < RL.TargetSpeed)
    
    RL.Status = 'AccOn';
    
    if (RL.AcclCount == 0)
        RL.AcclCount = AcclSectLength;
    else
        
    end
    
    RL.PercentageRun = (RL.TargetSpeed-RL.VSpeedIn)./RL.TargetSpeed;
    RL.AValReq = ((RL.TargetSpeed-RL.VSpeedIn)./RL.TargetSpeed)*CD.AMax;
    if RL.PercentageRun<MinAccel
        RL.PercentageRun=MinAccel;
    end
    
elseif RL.VSpeedIn >= RL.TargetSpeed
    RL.PercentageRun = 0;
    RL.Status = 'Coast';
    RL.AValReq = 0;
end

%% Torque Level
if  RL.PercentageRun<0
    RL.PercentageRun = 0;
    RL.Status = 'Coast';
end

RL.OutputTorque = RL.PercentageRun*CD.TMax;

[RL]= VehicleResp(RL,CD,RD);
[RL,Result] = PowerResp(RL,CD,Result);
end

