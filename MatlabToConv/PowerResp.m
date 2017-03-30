function [RL] = PowerResp(RL,CD)
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
    
    RL.DriveP = RL.RFTot*RL.VSpeedAv;%W
    RL.DriveT = RL.RFTot*CD.WheelRad;%Nm
    RL.MotorPOut = RL.OutputTorque*RL.ASpeed;%W
    OutputTorque = RL.OutputTorque;
    
    %Calculate Motor efficiency
    if OutputTorque >0 && OutputTorque<=10
        RL.MotorState='Running';
        RL.MotorEff = (-0.004420321*(OutputTorque^2)) +(0.106883204*OutputTorque) +	0.042251935;
    elseif OutputTorque > 10 && OutputTorque<=30
        RL.MotorState='Running';
        RL.MotorEff = (-0.000692995*(OutputTorque^2))+(0.03003544*OutputTorque)+0.456065934;
    elseif OutputTorque > 30 && OutputTorque<=37.5
        RL.MotorState='Running';
        RL.MotorEff = (-0.000535714*(OutputTorque^2))+(0.019857143*OutputTorque)+0.618;
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
        RL.ElecPIn = RL.MotorPIn/(CD.ElecEff1*CD.ElecEff2);
    else
        RL.ElecPIn=0;
        RL.MotorPIn=0;
        RL.MotorEff = 1.5;
        RL.Notes=char([RL.Notes ';Motor Calc Problem']);
    end
    
    %% Basic System
    if RL.ElecPIn>0&& RL.ElecPIn<=211.5
        RL.FuelConsumpcalc = 0.016973995*(RL.ElecPIn)-2.22045E-16;
        RL.FCState='Running';
    elseif RL.ElecPIn> 211.5 && RL.ElecPIn<=408
        RL.FuelConsumpcalc = 0.003053435*(RL.ElecPIn)+2.944198473;
        RL.FCState='Running';
        
    elseif RL.ElecPIn>408 && RL.ElecPIn<=1000
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
end

%% Updating for next loop
RL.VSpeedIn=RL.VSpeedEnd;
end

