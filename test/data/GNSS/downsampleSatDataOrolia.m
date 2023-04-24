format long

fp = "Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/";

file_id = fopen([fp 'whitelist.txt'], 'w');

fdisp(file_id, "Elapsed Time (ms)")

for i = 0:5:5*60 % in minutes
  t = 60000 + i * 60*1e3; % in ms
  fdisp(file_id, t);
end

fclose(file_id);

dirs = dir(fp);
for i = 1:length(dirs)
    if (endsWith(dirs(i).name, ".csv"))
      disp([num2str(i) "/" num2str(length(dirs)) " " dirs(i).name]);
      system([" awk -F, '(NR==FNR){a[$1];next}($1 in a)' '" fp "whitelist.txt' '" fp dirs(i).name "' > '" fp "output.csv'"]);
      delete([fp dirs(i).name])
      rename([fp "output.csv"], [fp dirs(i).name])
    endif
endfor

delete([fp 'whitelist.txt'])


