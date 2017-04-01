function [RL,Result] = PowerResp(RL,CD,Result)
%% Power Production
% The power production side
if strcmp(RL.Status, 'Coast')
    %FuelUsedConstant
    RL.MotorState= 'Coast';
    RL.FCState = 'Off';
    RL.FuelEfficiencyLocal = 0;
    RL.FuelEfficiencyLocalcalc = 0;
    RL.MotorEff = 1;%'N/A';
    RL.MotorPIn = 0;
    RL.MotorPOut = 0;
    RL.DriveP = 0;%W
    RL.DriveT = 0;%Nm
    RL.FCPIn = 0;
    RL.ElecPIn =0;
    RL.FCEff = 1;%'N/A';
    RL.FuelConsump = 0;
    RL.FuelConsumpcalc = 0; 
else
    
RL.DriveP = RL.RFTot.*RL.VSpeedAv;%W
RL.DriveT = RL.RFTot.*CD.WheelRad;%Nm
RL.MotorPOut = RL.OutputTorque.*RL.ASpeed;%W
OutputTorque = RL.OutputTorque;

%Calculate Motor efficiency
if OutputTorque >0 && OutputTorque<=10
    RL.MotorState='Running';
    RL.MotorEff = (-0.004420321.*(OutputTorque^2)) +(0.106883204.*OutputTorque) +	0.042251935;
elseif OutputTorque > 10 && OutputTorque<=30
    RL.MotorState='Running';
    RL.MotorEff = (-0.000692995.*(OutputTorque^2))+(0.03003544.*OutputTorque)+0.456065934;
     elseif OutputTorque > 30 && OutputTorque<=37.5
    RL.MotorState='Running';
    RL.MotorEff = (-0.000535714.*(OutputTorque^2))+(0.019857143.*OutputTorque)+0.618;
elseif OutputTorque>37.5 && OutputTorque<=38
    RL.MotorState='Running';
    RL.MotorEff = 0.35;
elseif OutputTorque>38
    RL.MotorState='Running';
    RL.MotorEff = 0.25;
elseif OutputTorque ==0 %Catching exceptions
    RL.MotorPIn =0;
    RL.MotorEff = 'N/A';
    RL.MotorState='PassedRules';
    RL.Notes=char([RL.Notes ';No torque, motor should be off']);
end

if strcmp(RL.MotorState,'Running')
%Power required by motor for required Torque
RL.MotorPIn = RL.MotorPOut/RL.MotorEff;
RL.ElecPIn = RL.MotorPIn/(CD.ElecEff1.*CD.ElecEff2);
else
    RL.ElecPIn=0;
    RL.MotorPIn=0;
    RL.MotorEff = 1.5;
    RL.Notes=char([RL.Notes ';Motor Calc Problem']);
end

%% Basic System

%Get FC efficiency
try
RL.FCEff = -9.904E-07.*(RL.ElecPIn^2)+ (0.001355043.*RL.ElecPIn) + 0.110436555;
%Fuel efficiency
RL.FCPIn = RL.ElecPIn/RL.FCEff; 
RL.FuelConsump = RL.FCPIn/178.61; %(L/min)
catch
    RL.Notes=char([RL.Notes '; Fuel Cell Power Too High']);
end

if RL.ElecPIn>0&& RL.ElecPIn<=211.5
%         Powercalc = 58.91364903*(FuelConsump)-1.42109E-14;
        RL.FuelConsumpcalc = 0.016973995*(RL.ElecPIn)-2.22045E-16;
        RL.FCState='Running';
    elseif RL.ElecPIn> 211.5 && RL.ElecPIn<=408
%         Powercalc = 327.5*(FuelConsump)-964.225;
        RL.FuelConsumpcalc = 0.003053435*(RL.ElecPIn)+2.944198473;
        RL.FCState='Running';

    elseif RL.ElecPIn>408 && RL.ElecPIn<=1000
%         Powercalc = 72.93900629*(FuelConsump)+166.8681678;
        RL.FuelConsumpcalc = 0.012403748*(RL.ElecPIn)-1.139490754;
        RL.FCState='Running';

    elseif RL.ElecPIn<=0
        RL.FCPIn = 0;
        RL.FCEff = 1.5;
        RL.FCState='PassedR';
        RL.FuelConsump = 0;
        RL.FuelConsumpcalc = 0;
        RL.Notes=char([RL.Notes ';Electrical Power too low']);
else
        RL.FCEff = 1.5;
        RL.FCState='OVRLDNA';
        RL.Notes=char([RL.Notes ';Fuel Cell Power Too High']);   
end
    if strcmp(RL.FCState,'Running');
            RL.FuelUsed = RL.FuelUsed + (RL.FuelConsump*(RL.TimeAdd/60));
            RL.FuelUsedcalc = RL.FuelUsedcalc +(RL.FuelConsumpcalc*(RL.TimeAdd/60));
            RL.FuelEfficiencyLocalcalc= 1/(RL.FuelConsumpcalc/(60*RL.VSpeedAv));%km/m^3
            RL.FuelEfficiencyLocal = 1/(RL.FuelConsump/(60*RL.VSpeedAv));%km/m^3
    else 
        RL.FuelConsump = 0;
        RL.FuelConsumpcalc=0;
        RL.FCEff = 1.5;
        RL.FCPIn = 0;
        RL.Notes=char([RL.Notes '; Fuel Cell error']);
    end
end

RL.FuelEfficiency = 1/((RL.FuelUsed./(RL.Clock./60))/(60*(RL.DistanceTotal./RL.Clock)));%km/m^3
RL.FuelEfficiencycalc = 1/((RL.FuelUsedcalc./(RL.Clock./60))/(60*(RL.DistanceTotal./RL.Clock)));%km/m^3

Result=[Result;transpose(struct2cell(RL))];

%% Updating for next loop
RL.VSpeedIn=RL.VSpeedEnd;
end

