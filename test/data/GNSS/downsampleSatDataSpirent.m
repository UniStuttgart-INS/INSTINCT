fp = "Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/";

fin = [fp "sat_data_V1A1.csv"];

format long

file_id = fopen([fp 'whitelist.txt'], 'w');

fdisp(file_id, "Build")
fdisp(file_id, "6")
fdisp(file_id, "Group A")
fdisp(file_id, "L1/E1/S/B1I")
%fdisp(file_id, "TOW_ms")
fdisp(file_id, "Time_ms")

for i = 0:5:5*60 % in minutes
  t = 60000 + i * 60*1e3; % in ms
  fdisp(file_id, t);
end

fclose(file_id);

system(["awk -F, '(NR==FNR){a[$1];next}($1 in a)' '" fp "whitelist.txt' '" fin "' > '" fp "output.csv'"]);

%delete([fp 'whitelist.txt'])
%delete(fin)
%rename([fp "output.csv"],fin)
