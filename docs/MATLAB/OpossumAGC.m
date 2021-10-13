%% OpossumAGC Automatic gain control with hysteresis control of noisy signals
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

clc; close all hidden; clearvars; %drawnow; pause(0.05);

volume = 34;
audio_level = 3452;
range_mB = 600;
range_step_mB = 50;

noiseline = @(start, stop, len, noiselevel) rot90((randn(1, len) .* noiselevel) ...
              + linspace(start, stop, len), -1);       
segment_length = @(x) round(abs(x * randn));

raw_audio_level = ([noiseline(3000, 4000, segment_length(1001), 30) 
                    noiseline(4000, 2800, segment_length(1001), 40)
                    noiseline(2800, 6500, segment_length(1001), 50)
                    noiseline(6500, 7000, segment_length(1001), 45)
                    noiseline(7000, 6300, segment_length(1001), 55)
                    noiseline(6300, 4000, segment_length(1001), 50)
                    noiseline(4000, 4500, segment_length(1001), 35)
                    noiseline(4500, 1800, segment_length(1001), 30)
                    noiseline(1800, 1700, segment_length(1001), 20)
                    noiseline(1700, 3600, segment_length(1001), 30)]);

mean_audio_level = zeros(size(raw_audio_level));
mbuff = raw_audio_level(1) * ones(1, 32);
buff_indx_32 = 1;
for k = 1:length(raw_audio_level)
    if (buff_indx_32 > 32)
        buff_indx_32 = 1;
    end
    if (raw_audio_level(k) > mbuff(buff_indx_32))
        mbuff(buff_indx_32) = mbuff(buff_indx_32) ...
            + (raw_audio_level(k) - mbuff(buff_indx_32)) / 2;
    else
        mbuff(buff_indx_32) = mbuff(buff_indx_32) .* (31/32);
    end
    mean_audio_level(k) = mean(mbuff);
    
    buff_indx_32 = buff_indx_32 + 1;
end

dB_CoefTable = [2053, 2175, 2303, 2440, 2584;
                2738, 2900, 3072, 3254, 3446;
                3651, 3867, 4096, 4339, 4596;
                4868, 5157, 5462, 5786, 6129;
                6492, 6876, 7284, 7715, 8173];
levels_raw = bitshift(uint32(audio_level .* dB_CoefTable), -12);

[v_map, a, b, C] = com.programming.OpossumVolumeMap(volume, range_mB, range_step_mB);

v = b:-1:a
C

%VolOut_absolute_mB = fliplr(rot90(reshape(c(v_map - a + 1), 5, 5), -1))
dBFastRelativeLevel_mB = levels_raw
VolOut_setting = fliplr(rot90(reshape(v_map - 1, 5, 5), -1))

% ##################################################################################################
[vol, volOut] = deal(volume);
DB_FAST_COEFFICIENT_COUNT = numel(dB_CoefTable);
levels_raw = sort(double(levels_raw(:)));


indx = 1 * ones(length(raw_audio_level), 1);
indx(1) = 13;
prev_indx = 1;
for m = 2:length(raw_audio_level)
  for k=DB_FAST_COEFFICIENT_COUNT:-1:1
    if (levels_raw(k) < mean_audio_level(m))
      if (abs(prev_indx - (k)) > 1)  
        indx(m) = (k);
        prev_indx = indx(m);
        break;
      else
        indx(m) = prev_indx;
        break;
      end
    end
  end
end

H = plotyy((1:length(raw_audio_level))./10, raw_audio_level, ...
           (1:length(raw_audio_level))./10, v_map(indx));
hold on;
plot(H(1), (1:length(raw_audio_level))./10, mean_audio_level, 'r');
L = line([zeros(1, 25); length(raw_audio_level) .* ones(1, 25)], [levels_raw.'; levels_raw.']);
set(L(13), 'lineWidth', 3);
set(H(1), 'YLim', [min(levels_raw), max(levels_raw)]);
set(H(2), 'YLim', [a, b]);
set(H(2), 'YGrid', 'on', 'YMinorGrid', 'off', 'YTick', 1, 'YMinorTick', 'on', 'YTickMode', 'auto');
A2 = get(H(2), 'Children');
set(A2(1), 'LineWidth', 2);

%figure(2); plot(indx)
