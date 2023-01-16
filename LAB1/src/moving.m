move = rosbag('move.bag');
moveSelectTopic = select(move,'Topic','/GPSDATA');
s_msgs = readMessages(move,'DataFormat','struct');
s_msgs{1}

s_Seq = cellfun(@(m) double(m.Header.Seq),s_msgs);
s_Latitude = cellfun(@(m) double(m.Latitude),s_msgs);
s_Longitude = cellfun(@(m) double(m.Longitude),s_msgs);
s_Altitude = cellfun(@(m) double(m.Altitude),s_msgs);
s_Zone = cellfun(@(m) double(m.Zone),s_msgs);
s_Letter = cellfun(@(m) double(m.Letter),s_msgs);
s_UtmEasting = cellfun(@(m) double(m.UtmEasting),s_msgs);
s_UtmNorthing = cellfun(@(m) double(m.UtmNorthing),s_msgs);

mean_utme = mean(s_UtmEasting)
mean_utmn = mean(s_UtmNorthing)
mean_a = mean(s_Altitude)
std_utme = std(s_UtmEasting)
std_utmn = std(s_UtmNorthing)
std_a = std(s_Altitude)

figure(1)
scatter(s_UtmEasting,s_UtmNorthing,'filled')
title('UTMeasting Vs UTMnorthing');
xlabel('UtmEasting');
ylabel('UtmNorthing');


figure(2)
scatter(s_Seq,s_Altitude,'filled')
title('SEQ Vs ALTITUDE');
xlabel('Seq');
ylabel('Altitude');



figure(3)
A = s_UtmEasting;
B = s_UtmNorthing;
hold on
plot(A,B)
p_UtmEasting = polyfit(A,B,1);
f_UtmEasting = polyval(p_UtmEasting,A);
plot(A,f_UtmEasting)
title('UTMeasting Vs UTMnorthing');
xlabel('UtmEasting');
ylabel('UtmNorthing');
hold off


X = s_Seq;
Y = s_Altitude;
figure(4)
hold on
plot(X,Y)
W = polyfit(X,Y,1);
R = polyval(W,X);
plot(X,R)
title('SEQ Vs ALTITUDE');
xlabel('Seq');
ylabel('Altitude');
hold off

error = zeros(size(s_UtmEasting));
for e = 1:size(s_UtmEasting)
    error(e) = abs(f(e)-s_UtmEasting(e));
end
error_min = min(error)
error_max = max(error)