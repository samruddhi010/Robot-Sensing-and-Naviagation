stat_clear_cen = rosbag('stat_clear_cen.bag');
stat_clear_cenSelectTopic = select(stat_clear_cen,'Topic','/gps_data_lab2');
s_msgs = readMessages(stat_clear_cenSelectTopic,'DataFormat','struct');
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



figure(4)
scatter(s_Seq,s_UtmEasting,'filled')
title('SEQ Vs UtmEasting');
xlabel('Seq');
ylabel('UtmEasting');

figure(5)
scatter(s_Seq,s_UtmNorthing,'filled')
title('SEQ Vs UtmNorthing');
xlabel('Seq');
ylabel('UtmNorthing');



