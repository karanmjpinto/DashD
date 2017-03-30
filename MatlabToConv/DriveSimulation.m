% % % function [ output_args ] = DriveSimulation( input_args )
%close all
%clear all

%TrackData = readtable('TrackData.txt','ReadVariableNames',true);
TrackData = readtable('TrackDataCoord.csv','ReadVariableNames',true);
%TrackData = readtable('TrackDataSimple.txt','ReadVariableNames',true);

RunLap = size(TrackData);
% Doubling for program
TrackData = [TrackData;TrackData;TrackData];

RaceData.Topo = TrackData; %Used to pass in track data, in python it is seperate

RaceData.LookAheadDist = 300;
RaceData.AcclSectLength = 15;
RaceData.MinAccel = 0.2;
RaceData.TargetSpeedIn=4;%m/s
RaceData.TargetSpeedConst=5.5;%m/s

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
CarData.TMax = 37; %Nm max torque from motor 37.5 really but questionable
CarData.MaxLatForce = 200; %N
CarData.BrakeDecel = 3; %M/s^2 (magnitude, should be positive)

%% Required inputs from Magic Pie
RaceLoop.VSpeedIn = 4.9;
RaceLoop.Grad = 0;
RaceLoop.Dist = 0;
RaceLoop.DistanceTotal = 0; %Starting at beginning
RaceLoop.Status = 'AccOn'; %'AccOn' or 'Coast'
RaceLoop.AcclCount = 0;
RaceLoop.TargetSpeed = 7.3;
RaceLoop.AValAvail = 0;

StartPosition = 1; %'i'

for i=StartPosition:(StartPosition+RunLap(1))%
    RaceLoop.i=i;
    RaceLoop.Position = TrackData.Distance(i);
    RaceLoop = Throttle(RaceLoop,CarData,RaceData);
    RaceLoop = VehicleResp(RaceLoop,CarData,RaceData);
    RaceLoop = PowerResp(RaceLoop,CarData);

    disp([num2str(RaceLoop.DistanceTotal), ': ', num2str(RaceLoop.TargetSpeed)]) %Position in Track
end
