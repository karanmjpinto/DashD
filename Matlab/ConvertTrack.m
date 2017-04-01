TrackData = importdata('TrackData(XtoY).csv');
TrackDataGPS = importdata('TrackCoOrds.csv');
RTrackDataGPS = TrackDataGPS;

%These fudge factors line them up better
TrackData(:,2) = TrackData(:,2)*1.08;
TrackData(:,3) = TrackData(:,3)*1.1;

close all
FinLine = [-0.0116944,	51.5376605;
    -0.0113511,	51.5377339];

LatDatum = mean(FinLine(:,1)); 
LongDatum = mean(FinLine(:,2));

%TrackDataGPS(:,1) = TrackDataGPS(:,1) - XOffset;
%TrackDataGPS(:,2) = TrackDataGPS(:,2) - YOffset;

for i = 1:numel(TrackDataGPS(:,1))
    %Convert long/lat to cartisian co-ordinates, finish line is 0,0
    %https://en.wikipedia.org/wiki/Geographic_coordinate_system
    %http://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
    %latMid = %TrackDataGPS(i,1)/2.0;
    longMid = TrackDataGPS(i,2);
    m_per_deg_lat = 111132.954 - 559.822 * cosd( 2.0 * longMid ) + 1.175 * cosd( 4.0 * longMid);
    m_per_deg_lon = (3.14159265359/180 ) * 6378137 * cosd ( longMid );
    
    TrackDataGPS(i,1) = m_per_deg_lon*(TrackDataGPS(i,1) - LatDatum);%*0.57;
    TrackDataGPS(i,2) = m_per_deg_lat*(TrackDataGPS(i,2) - LongDatum);%*0.93;
    
    %Add Z position to GPS data
    comp = abs([TrackData(:,2) - TrackDataGPS(i,1),TrackData(:,3) - TrackDataGPS(i,2),TrackData(:,4)]);
    comp(:,4) = sqrt(comp(:,1).^2 + comp(:,2).^2);
    
    [M,I] = min(comp(:,4));
    comp = [comp(end,:);comp;comp(1,:)];
    
    if (comp(I,4) > comp(I+2,4))
        Iother = I+1;
    else
        Iother = I-1;
    end
    
    if (Iother == 0)
        Iother = numel(TrackData(:,1));
    elseif (Iother > numel(TrackData(:,1)))
        Iother = 1;
    end
    
    %Select Proxist values
    Proxi = [TrackData(I,:); TrackData(Iother,:)];
    dataDist = sqrt((Proxi(1,2) - Proxi(2,2))^2 + (Proxi(1,3) - Proxi(2,3))^2);
    interDist = sqrt((Proxi(1,2) - TrackDataGPS(i,1))^2 + (Proxi(1,3) - TrackDataGPS(i,2))^2);
    
    TrackDataGPS(i,3) =  Proxi(1,4) + ((Proxi(2,4) - Proxi(1,4))*(interDist/dataDist));
end

%TrackDataGPS(:,1) = TrackDataGPS(:,1)/0.55;
%TrackDataGPS(:,2) = TrackDataGPS(:,2)/0.93;

length = 0;
for i = 2:numel(TrackDataGPS(:,1))
    length = length + sqrt((TrackDataGPS(i,1) - TrackDataGPS(i-1,1)).^2....
            + (TrackDataGPS(i,2) - TrackDataGPS(i-1,2)).^2....
            + (TrackDataGPS(i,3) - TrackDataGPS(i-1,3)).^2);
end

disp(['Track length: ',num2str(length)]);
hold on
plot(TrackData(:,2),TrackData(:,3),'r')
plot(TrackDataGPS(:,1),TrackDataGPS(:,2),'b')
plot3(TrackData(:,2),TrackData(:,3),TrackData(:,4),'r')
plot3(TrackDataGPS(:,1),TrackDataGPS(:,2),TrackDataGPS(:,3),'b')


TrackDataGPS = [RTrackDataGPS(:,1:2), TrackDataGPS];
TrackDataGPS = [TrackDataGPS(end,:);TrackDataGPS];

dist = 0;
for i = 2:numel(TrackDataGPS(:,1))
    TrackDataGPS(i,6) = (TrackDataGPS(i-1,5) - TrackDataGPS(i,5))/sqrt((TrackDataGPS(i,3) - TrackDataGPS(i-1,3))^2 + (TrackDataGPS(i,4) - TrackDataGPS(i-1,4))^2);
    TrackDataGPS(i,7) = tand(TrackDataGPS(i,6));
    
    u = TrackDataGPS(i-1,3:5);
    v = TrackDataGPS(i,3:5);
    TrackDataGPS(i,8) = atan2d(norm(cross(u,v)),dot(u,v));
    
    travel = sqrt((TrackDataGPS(i,3) - TrackDataGPS(i-1,3)).^2....
            + (TrackDataGPS(i,4) - TrackDataGPS(i-1,4)).^2....
            + (TrackDataGPS(i,5) - TrackDataGPS(i-1,5)).^2);
    length = length + travel;
    TrackDataGPS(i,9) = travel;
    TrackDataGPS(i,10) = length;
end
TrackDataGPS(1,:) = [];

csvwrite('ALLTrackData.csv',TrackDataGPS);
