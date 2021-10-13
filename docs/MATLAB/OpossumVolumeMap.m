function [v_map, a, b, C] = OpossumVolumeMap(volume, range_mB, range_step_mB)
%% OpossumVolumeMap generates a volume map v_map for a given input volume and range
%
% Copyright (c) 2017-2021 Winry R. Litwa-Vulcu. All rights reserved.
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <https://www.gnu.org/licenses/>.

  volume(volume < 1) = 1;
  volume(volume > 64) = 64;

  offset_mB = linspace(0, 2 * range_mB, round((2 * range_mB)/50) + 1);  

  gain_mB = [-10950;   -9290;    -9030;    -8680;
             -8430;    -8080;    -7830;    -7470;
             -7220;    -6870;    -6620;    -6270;
             -6020;    -5670;    -5420;    -5060;
             -4810;    -4560;    -4370;    -4210;
             -3960;    -3760;    -3600;    -3340;
             -3150;    -2980;    -2720;    -2520;
             -2350;    -2160;    -1970;    -1750;
             -1640;    -1540;    -1440;    -1310;
             -1200;    -1090;     -990;     -890;
              -710;     -600;     -500;     -340;
              -190;      -50;       50;      120;
               160;      200;      240;      290;
               340;      390;      440;      490;
               540;      590;      650;      700;
               760;      820;      880;      950];

  % determine the levels within the range
  a = 1;
  b = 64;
  if (volume ~= 1)
    for k = volume-1:-1:1
      if ((gain_mB(k) - gain_mB(volume)) <  -range_mB);
        a = k + 1;
        break;
      end
    end
  end
  if (volume ~= numel(gain_mB))
    for k = volume+1:numel(gain_mB);
      if ((gain_mB(k) - gain_mB(volume)) >  range_mB)
        b = k - 1;
        break;
      end
    end
  end

  v = b:-1:a;
  sizeof_v = numel(v);
  c_normal = (gain_mB(v) - gain_mB(volume));

	v_map = zeros(25, 1);
  skip_zero_indx = 0;
  map_indx = 1;
  for k=1:numel(offset_mB)
    if (c_normal(map_indx) == 0)
      skip_zero_indx = 1;
    end
    if ((map_indx + skip_zero_indx) <= sizeof_v)
      if ((range_mB - c_normal(map_indx + skip_zero_indx)) < ((k - 1) * range_step_mB))
        map_indx = map_indx + 1;
      end
    end
    v_map(k) = v(map_indx);
  end
  
  C = [gain_mB(v), c_normal, (range_mB - c_normal)];
end
