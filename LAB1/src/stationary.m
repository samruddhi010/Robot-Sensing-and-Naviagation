name = rosbag('name.bag');
nameSelectTopic = select(name,'Topic','/GPSDATA');
n_msgs = readMessages(name,'DataFormat','struct');
n_msgs{1}

n_Seq = cellfun(@(m) double(m.Header.Seq),n_msgs);
n_Latitude = cellfun(@(m) double(m.Latitude),n_msgs);
n_Longitude = cellfun(@(m) double(m.Longitude),n_msgs);
n_Altitude = cellfun(@(m) double(m.Altitude),n_msgs);
n_Zone = cellfun(@(m) double(m.Zone),n_msgs);
n_Letter = cellfun(@(m) double(m.Letter),n_msgs);
n_UtmEasting = cellfun(@(m) double(m.UtmEasting),n_msgs);
n_UtmNorthing = cellfun(@(m) double(m.UtmNorthing),n_msgs);

mean_utmea = mean(n_UtmEasting)
mean_utmno = mean(n_UtmNorthing)
mean_al = mean(n_Altitude)
std_utmea = std(n_UtmEasting)
std_utmno = std(n_UtmNorthing)
std_al = std(n_Altitude)


figure(5)
scatter(n_Seq,n_UtmNorthing,'filled')
title('SEQ Vs UTMnorthing');
xlabel('SEQ');
ylabel('UtmNorthing');

figure(6)
scatter(n_Seq,n_UtmEasting,'filled')
title('SEQ Vs UTMeasting');
xlabel('SEQ');
ylabel('Utmeasting');


figure(1)
scatter(n_UtmEasting,n_UtmNorthing,'filled')
title('UTMeasting Vs UTMnorthing');
xlabel('UtmEasting');
ylabel('UtmNorthing');


figure(2)
scatter(n_Seq,n_Altitude,'filled')
title('SEQ Vs ALTITUDE');
xlabel('Seq');
ylabel('Altitude');



figure(3)
A = n_UtmEasting;
B = n_UtmNorthing;
hold on
plot(A,B)
p = polyfit(A,B,1);
f = polyval(p,A);
title('UTMeasting Vs UTMnorthing');
xlabel('UtmEasting');
ylabel('UtmNorthing');
plot(A,f)
hold off


X = n_Seq;
Y = n_Altitude;
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