function [RL] = VehicleResp(RL,CD,RD)
%% Vehicle Response
% Vehicle response to torque levels

%% Dynamic Updates
% Position, Lap data and distance data
TrackData = RD.Topo;
RL.Position = TrackData.Distance(RL.i); %Position

RL.PortionOfTrack = [num2str(RL.Position) ' - ' num2str(RL.Position +1)]; % Print Position

RL.Notes = char(['Notes:' RL.PortionOfTrack ', ' RL.Notes]);

RL.Dist = TrackData.d(RL.i);

RL.DistanceTotal = RL.DistanceTotal + RL.Dist;

% Input track data from array TrackData
RL.Grad = TrackData.angle(RL.i);

%% Resistances
% Static Resistance
RL.RollR = CD.M*CD.g*CD.CR*cos(deg2rad(RL.Grad));%N % assuming the same rolling resistance for each wheel      (checked)
RL.GradientR = CD.M*CD.g*sin(deg2rad(RL.Grad));% N            (checked)
RL.StaticR = RL.RollR + RL.GradientR;

%% Solving for available Torque and Acceleration
if strcmp(RL.Status,'Coast')
    % torque version
    QA = (CD.M*CD.WheelRad+(CD.IWheelBack +(2*CD.IWheelFront))./CD.WheelRad+(RL.Dist*(0.25)*CD.rhoair*CD.WheelRad*CD.Cd*CD.FrontA))^2;
    QB = (-2*(-RL.StaticR*CD.WheelRad-((0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2)))*(CD.M*CD.WheelRad+(CD.IWheelBack +(2*CD.IWheelFront))./CD.WheelRad+(RL.Dist*(0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA)))-((1./8)*RL.Dist*(CD.Cd*CD.rhoair*CD.WheelRad*CD.FrontA*RL.VSpeedIn)^2);
    QC = ((-RL.StaticR*CD.WheelRad-((0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2)))^2)-((1./4)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2))^2;

    RL.AValAvail = (-QB-sqrt((QB^2)-(4*QA*QC)))/(2*QA);
else
    % torque version
    QA = (CD.M*CD.WheelRad+(CD.IWheelBack +(2*CD.IWheelFront))./CD.WheelRad+(RL.Dist*(0.25)*CD.rhoair*CD.WheelRad*CD.Cd*CD.FrontA))^2;
    QB = (-2*((RL.OutputTorque)-RL.StaticR*CD.WheelRad-((0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2)))*(CD.M*CD.WheelRad+(CD.IWheelBack +(2*CD.IWheelFront))./CD.WheelRad+(RL.Dist*(0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA)))-((1./8)*RL.Dist*(CD.Cd*CD.rhoair*CD.WheelRad*CD.FrontA*RL.VSpeedIn)^2);
    QC = (((RL.OutputTorque)-RL.StaticR*CD.WheelRad-((0.25)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2)))^2)-((1./4)*CD.rhoair*CD.WheelRad* CD.Cd* CD.FrontA*(RL.VSpeedIn^2))^2;
    
    % Available Acceleration and average Speed updates
    RL.AValAvail = (-QB-sqrt((QB^2)-(4*QA*QC)))/(2*QA);
end

RL.VSpeedEnd =  sqrt((RL.VSpeedIn.^(2)) + 2*RL.AValAvail*RL.Dist); %m/s
RL.VSpeedAv = (RL.VSpeedEnd+RL.VSpeedIn)./2;

% Rotational speeds and accelerations
RL.RaVal = RL.AValAvail./(CD.WheelRad);% rad/s^2
RL.RSpeed = RL.VSpeedAv*60./(pi*CD.WheelDiam);%RPM
RL.ASpeed = RL.RSpeed*2*pi./60; %Rad/s


%% Resistances
% Dynamic Resistances
RL.AeroR = (0.5)*CD.rhoair* CD.Cd* CD.FrontA*(RL.VSpeedAv.^2); % N        (checked)
RL.DriveR = CD.M*RL.AValAvail;% N              (checked)
RL.InertRTorque = RL.RaVal*(CD.IWheelBack +(2*CD.IWheelFront));% Nm       (checked)
RL.InertR = RL.InertRTorque* CD.WheelRad;% N           (checked)

% Total Resistance
RL.RFTot = RL.StaticR + RL.AeroR + RL.DriveR + RL.InertR;% N
end

