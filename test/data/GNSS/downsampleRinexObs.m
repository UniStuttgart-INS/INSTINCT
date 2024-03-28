fin = "Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Spirent_RINEX_MO.obs";
%fin = 'Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/SkydelRINEX_S_20230080959_04H_02Z_MO.rnx';

fid_in = fopen(fin);
fid_out = fopen('output.txt', 'w');

add = true;
while (true)
  str = fgetl(fid_in);
  if (str == -1)
    break;
  endif

  if (startsWith(str,">"))
%    year=str(3:6);
%    mon=str(8:9);
%    day=str(11:12);
%    hour=str(14:15);
    min=str2double(str(17:18));
    sec=str2double(str(20:23));
    if sec == 60
      sec = 0;
      min = min + 1;
    endif
    if (mod(min, 5) == 0 && sec == 0)
      add = true;
    else
      add = false;
    endif
  endif

  if (add == true)
    fdisp(fid_out, str);
  endif
endwhile

fclose(fid_in);
fclose(fid_out);

%delete(fin)
%rename("output.txt", fin)
