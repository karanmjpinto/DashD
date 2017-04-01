%Produces analysed and compiled track data
%Cols of resulting Pad variable
%1 - Index, for usefullness
%2,3 - GPS Long and Lat
%4 - Z data interpolated from shell data
%5,6 - X and Y in meters, finish line as 0,0
%7 - Distance along track (cumulative)
%8 - Distance between point and next
%9 - Radius of this point and neighbours (in X,Y)
%10 - Gradient (Z/Dist)
%11 - Angle to vertical

%Shell track data, first and last point are duplicate Z, but with 0 and
%1659
%distance
ShellTrackData = importdata('TrackDataV2.csv');

%Google GPS co-ordinates
TrackDataGPS = importdata('TrackCoOrds.csv');

%Add duplicate point at end of data
%All data points are between the point and the next
TrackDataGPS = [TrackDataGPS; TrackDataGPS(1,:)];

%Corner cut-off
CornThres = 100;

close all
FinLine = [-0.01144, 51.53753;
   -0.01169, 51.53741];
LatDatum = mean(FinLine(:,1));
LongDatum = mean(FinLine(:,2));

for i = 1:(numel(TrackDataGPS(:,1))-1)
    %Convert long/lat to cartisian co-ordinates, finish line is 0,0
    %https://en.wikipedia.org/wiki/Geographic_coordinate_system
    %http://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
    %latMid = %TrackDataGPS(i,1)/2.0;
    longMid = TrackDataGPS(i,2);
    m_per_deg_lat = 111132.954 - 559.822 * cosd( 2.0 * longMid ) + 1.175 * cosd( 4.0 * longMid);
    m_per_deg_lon = (3.14159265359/180 ) * 6378137 * cosd ( longMid );
    
    TrackDataGPS(i,4) = m_per_deg_lon*(TrackDataGPS(i,1) - LatDatum);
    TrackDataGPS(i,5) = m_per_deg_lat*(TrackDataGPS(i,2) - LongDatum);
end
TrackDataGPS(end,:) = [];

%Arrange data so finish line is first point
[M,I] = min(abs(TrackDataGPS(1:end,4) + TrackDataGPS(1:end,5)));
TrackDataGPS = [TrackDataGPS(I:end-1,:); TrackDataGPS(1:I-1,:)];
TrackDataGPS = [TrackDataGPS(end,:); TrackDataGPS; TrackDataGPS(1,:)];

%Add distance, travel and radius data
length = 0;
TrackDataGPS(1,6) = 0;
for i = 2:(numel(TrackDataGPS(:,1)) - 1)
    %Add travel (d) data (only based on X and Y)
    travel = sqrt((TrackDataGPS(i+1,4) - TrackDataGPS(i,4)).^2....
            + (TrackDataGPS(i+1,5) - TrackDataGPS(i,5)).^2);
    TrackDataGPS(i,7) = travel;
    
    length = length + travel;
    TrackDataGPS(i+1,6) = length; %(Distance)

    %J sol
    [R,XC,YC] = circfit(TrackDataGPS(i-1:i+1,4),TrackDataGPS(i-1:i+1,5));
    if R > 200
        R = 200;
    end
    TrackDataGPS(i,8) = R;
end

isCorner = false;
swaps = [2];
for i = 2:(numel(TrackDataGPS(:,1)) - 1)
    if (isCorner ~= (TrackDataGPS(i,8) > CornThres))
        swaps(end+1) = i;
        isCorner = ~isCorner;
    end
end

swaps(end+1) = numel(TrackDataGPS(:,1));
for i = 2:2:(numel(swaps))
    TrackDataGPS(swaps(i-1):(swaps(i)-1),8) = mean(TrackDataGPS(swaps(i-1):(swaps(i)-1),8));
    if (i+1 <= numel(swaps))
        TrackDataGPS(swaps(i):(swaps(i+1)-1),8) = NaN;
    end
end

%Pad data
Pad = [];
for i = 2:(numel(TrackDataGPS(:,1)) - 1)
    Dist = TrackDataGPS(i,7);
    stepDist = (Dist/ceil(Dist/2));
    Pad = [Pad; TrackDataGPS(i,:)];
    Pad(end,7) = stepDist;
    for ii = 1:floor(Dist/2)
        DataDist = (TrackDataGPS(i,6)+(stepDist*(ii)));
        Pad = [Pad; 0, 0, 0, 0, 0, DataDist, stepDist, 0];
        
        for column = [1,2,4,5]
            Pad(end,column) = interp1(TrackDataGPS(2:end-1,6),TrackDataGPS(2:end-1,column),DataDist,'spline');
        end
        Pad(end,8) = interp1(TrackDataGPS(2:end-1,6),TrackDataGPS(2:end-1,8),DataDist,'linear');
    end
end

%Add height data
for i = 1:numel(Pad(:,1))
    Pad(i,3) = interp1(ShellTrackData(:,1)', ShellTrackData(:,2)', Pad(i,6),'spline');
end

%Add extra rows EITHER SIDE of data
Pad = [Pad(end,:); Pad; Pad(1,:)];

%Add gradient, angle and radius
for i = 2:(numel(Pad(:,1)) - 1)
    Pad(i,9) = (Pad(i+1,3) - Pad(i,3))/Pad(i+1,7); %Grad
    Pad(i,10) = atand(Pad(i,9)); %angle
end

%Crop first and last points
Pad(1,:) = [];
Pad(end,:) = [];

%Plot to show whats occuring
cols = []; 
for i = 1:numel(Pad(:,3))
    if isnan(Pad(i,8))
        cols(end+1,:) = [0,0,0];
    else
        cols(end+1,:) = num2col(CornThres - Pad(i,8),0,CornThres);
    end
end
scatter3(Pad(:,4),Pad(:,5),Pad(:,3),36,cols)
figure(2)
plot(Pad(:,6),Pad(:,8))

%Add index
Pad = [[0:numel(Pad(:,1))-1]',Pad];

%Save the data
dlmwrite('thisisit.csv',Pad,'precision', 12); 

%Test length
TLength = 0;
for i = 1:(numel(Pad(:,1)) - 1)
    %Add travel (d) data (only based on X and Y)
    travel = sqrt((Pad(i+1,4) - Pad(i,4)).^2....
            + (Pad(i+1,5) - Pad(i,5)).^2....
            + (Pad(i+1,6) - Pad(i,6)).^2);
    
    TLength = TLength + travel;
end