% % % function [ output_args ] = DriveSimulation( input_args )
%close all
%clear all

%TrackData = readtable('TrackData.txt','ReadVariableNames',true);
TrackData = readtable('TrackDataCoord.csv','ReadVariableNames',true);
%TrackData = readtable('TrackDataSimple.txt','ReadVariableNames',true);

RunLap = size(TrackData);
% Doubling for program
TrackData = [TrackData;TrackData];

RaceData.Laps = 8;
RaceData.Topo = TrackData;
RaceData.TrackLength = 2244;% m
RaceData.RaceDistance = RaceData.TrackLength.*8;% m
RaceData.TimeLim = 43*60; %seconds
RaceData.StndTrkSpeed = 7.3;

CarData.M = 50+50;%kg
CarData.FrontA = 0.393;%m^2
CarData.WheelDiam = 0.478;% m
CarData.WheelRad = CarData.WheelDiam./2;% m
CarData.WheelFrontM = 0.8226;% kg
CarData.WheelBackM = 5.26662;% kg
CarData.WheelCirc = pi.*CarData.WheelDiam;%m
CarData.IWheelBack = 0.06403299; %kgm2
CarData.IWheelFront = 0.039674395; %kgm2
CarData.ElecEff1=0.96;
CarData.ElecEff2=0.96;
CarData.g = 9.81;% m/s^2
CarData.rhoair = 1.23; % kg/m^3
CarData.CR = 0.00162; %0.004;% Rolling Resistance Coeff.
CarData.Cd = 0.08; % Drag Coefficient


%%Iteratives
RaceData.TargetSpeedIn=4;%m/s
RaceData.TargetSpeedConst=5.5;%m/s

CarData.TMax = 37; %Nm max torque from motor 37.5 really but questionable
CarData.AMax = 0.9; % m/s2 max acceleration
CarData.Throttle1 =0.8;


%% Required inputs from Magic Pie
RaceLoop.VSpeedIn = 4.9; %Start from stand still
RaceLoop.Clock = 0; %Start Clock
RaceLoop.DistanceTotal = 0; %Starting at beginning
RaceLoop.FuelUsed = 0;
RaceLoop.FuelUsedcalc =0;
RaceLoop.FirstAcc = 'N';% 'Y'
RaceLoop.TargetMet ='N';% 'Y'
RaceLoop.AvStable ='X';% 'Y' or 'N'
RaceLoop.Override='N'; % 'Y'
RaceLoop.Status = 'AccOn'; %'AccOn' or 'Coast'
RaceLoop.Notes = ''; % Error messagse
RaceLoop.AcclCount = 0;
RaceLoop.TargetSpeed = 7.3;
StartPosition = 1; %'i'


Result = [{'VSpeedIn'},{'Clock'},{'DistanceTotal'}, {'FuelUsed'} ,{'FuelUsedcalc'}, {'FirstAcc'}, {'TargetMet'}, {'AvStable' },{'Override'},{'Status'},{'Notes'},{'AcclCount'} ,{'LapTime'}, {'LapState'}, {'i'}, {'Position'}, {'TargetSpeed'}, {'VRaceAverage'},{'VLapAverage'} ,{'PercentageRun'}, {'AValReq' } ,{ 'OutputTorque' } ,{'PortionOfTrack'} ,  {'LapDistRem'},{'Dist'}, {'Grad'} , {'RollR'},{'GradientR'},{'StaticR'},{'AValAvail'} , {'VSpeedEnd'} , {'VSpeedAv'},{'TimeAdd'},{'TimeRem' }, {'LapTimeRem'}, {'RaVal'},{'RSpeed'},{'ASpeed'},{'AeroR'}, {'DriveR'}, {'InertRTorque'}, {'InertR' }, {'RFTot'},{'DriveP'},{'DriveT'},{'MotorPOut'},{'MotorState'},{'MotorEff'},{'MotorPIn'},{'ElecPIn'},{'FCEff'},{'FCPIn'} ,{'FuelConsump'} ,{'FuelConsumpcalc'}, {'FCState'}, {'FuelEfficiencyLocalcalc'} , {'FuelEfficiencyLocal'},{'FuelEfficiency'}, {'FuelEfficiencycalc'} ];
LapNumbers={'1';'2';'3';'4';'5';'6';'7';'8'};
LapTimes = [0;0;0;0;0;0;0;0];
LapState = ['Nbegun';'Nbegun';'Nbegun';'Nbegun';'Nbegun';'Nbegun';'Nbegun';'Nbegun'];
VLapAverage = [0;0;0;0;0;0;0;0];
LapTable = table(LapTimes,VLapAverage,LapState,'RowNames',LapNumbers);


for LapNo=1:1%RaceData.Laps
    LapNo; % Print Lap No
    RaceLoop.LapTime = 0;
    RaceLoop.LapState='YStart';
    LapTable(num2str(LapNo),3)= {'NEnded'};
    for i=StartPosition:(StartPosition+RunLap(1))%
        RaceLoop.i=i;
        RaceLoop.Position=i;
        [RaceLoop, Result] = Throttle(RaceLoop,CarData,RaceData,Result);
        disp([num2str(RaceLoop.DistanceTotal), ': ', num2str(RaceLoop.TargetSpeed)]) %Position in Track
    end
    LapState='YEnded';
    LapTable(num2str(LapNo),1)= {RaceLoop.LapTime};
    LapTable(num2str(LapNo),2)= {RaceLoop.VLapAverage};
    LapTable(num2str(LapNo),3)= {'YEnded'};
end


%Print all of it here for the 8 laps ....
%% Only for testing purposes
FullR = cell2table(Result(2:end,:));
FullR.Properties.VariableNames = Result(1,:);

Clock_mins=RaceLoop.Clock./60;
FuelEfficiency_km_m3 = RaceLoop.FuelEfficiency;
VRaceAverage_m_s =RaceLoop.VRaceAverage;
LapTable
OverallResults = table(Clock_mins, FuelEfficiency_km_m3,VRaceAverage_m_s)

%% Plotting Not required for App
Graphs= figure;
hold on
agrp=subplot(2,2,1);
plot(agrp,FullR.DistanceTotal,FullR.VSpeedIn);
hold on
plot(agrp,FullR.DistanceTotal,FullR.VSpeedAv);
ylabel('V (m/s)')
xlabel('dist (m)')
title('Starting Speed (and Average Speed)')
hold on
bgrp=subplot(2,2,2);
plot(bgrp,FullR.DistanceTotal,FullR.ElecPIn);
hold on
plot(bgrp,FullR.DistanceTotal,FullR.FCPIn);
hold on
plot(bgrp,FullR.DistanceTotal,FullR.MotorPOut);
hold on
plot(bgrp,FullR.DistanceTotal,FullR.MotorPIn);
% ylabel('H2 Power Used (W)')
ylabel('Fuel Cell P Out/In(W)')
xlabel('dist (m)')
title('Fuel Cell Power Out/In')
% title('Fuel Cell Power In')
% plot(bgrp,FullR.DistanceTotal,FullR.FCPInPlan);
hold on
cgrp=subplot(2,2,3);
plot(cgrp,FullR.DistanceTotal,FullR.FuelEfficiency);
hold on
plot(cgrp,FullR.DistanceTotal,FullR.FuelEfficiencycalc);
ylabel('Fuel Eff (km/m3)')
xlabel('dist (m)')
title('Fuel Eff')
% plot(cgrp,FullR.DistanceTotal,FullR.FuelEfficiencyPlan);
hold on
dgrp=subplot(2,2,4);
plot(dgrp,FullR.DistanceTotal,FullR.AValAvail);
hold on
%plot(dgrp,FullR.DistanceTotal,FullR.FCEff);
hold on
plot(dgrp,FullR.DistanceTotal,FullR.MotorEff);
ylabel('a (m/s2)')
xlabel('dist (m)')
title('Acceleration')

% % % end

