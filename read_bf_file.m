%READ_BF_FILE Reads in a file of beamforming feedback logs.
% Original version using the *C* version of read_bfee, compiled with
%   MATLAB's MEX utility, by
% (c) 2008-2011 Daniel Halperin <dhalperi@cs.washington.edu>
%
% New MATLAB only version with additional data entries by
% (c) 2024 Brendan Dijkstra <b.j.dijkstra.2@student.rug.nl>
%
% Way to "read" raw hex data using powershell:
%   format-hex "C:\filepath\csi_rx.dat" | more
% Or
%   format-hex "C:\filepath\csi_rx.dat" >> "C:\filepath\csi_rx.txt"
%
function ret = read_bf_file(filename)
%% Input check
error(nargchk(1,1,nargin));

%% Open file
f = fopen(filename, 'rb');
if (f < 0)
    error('Couldn''t open file %s', filename);
    return;
end

status = fseek(f, 0, 'eof');
if status ~= 0
    [msg, errno] = ferror(f);
    error('Error %d seeking: %s', errno, msg);
    fclose(f);
    return;
end
len = ftell(f);

status = fseek(f, 0, 'bof');
if status ~= 0
    [msg, errno] = ferror(f);
    error('Error %d seeking: %s', errno, msg);
    fclose(f);
    return;
end

%% Initialize variables
% Holds the return values - 1x1 CSI is 95 bytes big, so this should be upper bound
ret(ceil(len/95)) = struct("timestamp_low",[],"bfee_count",[], ...
               "Nrx",[],"Ntx",[],"rssi_a",[],"rssi_b",[], ...
               "rssi_c",[],"noise",[],"agc",[],"perm",[], ...
               "rate",[],"csi",[],"tv_sec",[],"tv_usec",[], ...
               "mac",'',"frame_count",[]);
cur = 0;                        % Current offset into file
count = 0;                      % Number of records output
broken_perm = 0;                % Flag marking whether we've encountered a broken CSI yet
triangle = [1 3 6];             % What perm should sum to for 1,2,3 antennas

%% Process all entries in file
% Need 3 bytes -- 2 byte size field and 1 byte code
while cur < (len - 3)
    % Read size and code
    field_len = fread(f, 1, 'uint16', 0, 'ieee-be');
    code = fread(f,1);
    cur = cur+3;

    % If unhandled code, skip (seek over) the record and continue
    if (code == 187) % get beamforming or phy data
        bytes = fread(f, field_len-1, 'uint8=>uint8');
        cur = cur + field_len - 1;
        if (length(bytes) ~= field_len-1)
            ret = ret(1:count); % shorten back from predefined length to here
            fclose(f);
            warning('Stopped importing early on CSI file (%s), at index %i out of excpected %i', filename, count, ceil(len/95));
            return;
        end
    else % skip all other info
        fseek(f, field_len - 1, 'cof');
        cur = cur + field_len - 1;
        continue;
    end
    
    if (code == 187) %hex2dec('BB')) Beamforming matrix -- output a record
        count = count + 1;

        ret(count) = read_bfee_local(bytes);
        perm = ret(count).perm;
        Nrx = ret(count).Nrx;
        if Nrx == 1 % No permuting needed for only 1 antenna
            continue;
        end
        if sum(perm) ~= triangle(Nrx) % matrix does not contain default values
            if broken_perm == 0
                broken_perm = 1;
                warning('WARN ONCE: Found CSI (%s) with Nrx=%d and invalid perm=[%s]\n', filename, Nrx, int2str(perm));
            end
        else
            ret(count).csi(:,perm(1:Nrx),:) = ret(count).csi(:,1:Nrx,:);
        end
    end
end
ret = ret(1:count); % shorten back from predefined length to here

%% Close file
fclose(f);
end

function ret = read_bfee_local(bytes)
    % Initialize variables from bytes
    timestamp_low = typecast(bytes(1:4), 'uint32');
    bfee_count = typecast(bytes(5:6), 'uint16');
    Nrx =bytes(9);
    Ntx = bytes(10);
    rssi_a = bytes(11);
    rssi_b = bytes(12);
    rssi_c = bytes(13);
    noise = bytes(14);
    agc = bytes(15);
    antenna_sel = bytes(16);
    len = typecast(bytes(17:18), 'uint16');
    fake_rate_n_flags = typecast(bytes(19:20), 'uint16');

    % Check that length matches what it should
    calc_len = (30 * (double(Nrx) * double(Ntx) * 8 * 2 + 3) + 6) / 8;
    if len ~= calc_len
        error('read_bfee:Size', 'Wrong beamforming matrix size. len: %d, calc_len: %d', len, calc_len);
    end
    
    tv_sec = typecast(bytes(21:24), 'uint32');
    tv_usec = typecast(bytes(25:28), 'uint32');
    %mac = reshape(rot90(dec2hex(bytes(29:34)),1),1,[]); % store bytes as chars
    mac = reshape(transpose(dec2hex(bytes(29:34))),1,[]); % store bytes as chars
    frame_count = typecast(bytes(35:38), 'uint32');

    % Process CSI
    payload = (bytes(39:end));

    % Initialize the CSI matrix
    csi = complex(zeros(Ntx, Nrx, 30, 'double'));
    
    % Index initialization
    index = 0;
    
    % Loop over each of the 30 subcarriers
    for subcarrierIndex = 1:30
        index = index + 3; % Skip the first 3 bytes as per the original logic
        remainder = mod(index,8);
        for j = 1:double(Nrx * Ntx)
            % Calculate byte position in payload
            byte_pos = floor(index / 8) + 1;

            % Process the real part
            % The actual data is partially in two bytes, so we take the
            % portions of each byte, shift accordingly, and add them.
            part1 = bitshift(payload(byte_pos), -remainder,"uint8");
            part2 = bitshift(payload(byte_pos+1), 8 - remainder,"uint8");

            % Combine for total real part
            % In this case the same as simply adding both positive ints.
            realPart = bitor(part1, part2);

            % Translate to signed ints, this turns any value above 2^7=128
            % into the negative region.
            realPart = typecast(realPart,'int8');

            % Process the imaginary part (similar process)
            part1_imag = bitshift(payload(byte_pos+1), -remainder,"uint8");
            part2_imag = bitshift(payload(byte_pos+2), 8 - remainder,"uint8");
            imagPart = bitor(part1_imag, part2_imag);
            imagPart = typecast(imagPart,'int8');

            % Calculate positions in the csi array
            txIndex = ceil(j / double(Nrx));
            rxIndex = mod(j-1, double(Nrx)) + 1;

            % Assign the combined real and imaginary parts to the csi array
            csi(txIndex, rxIndex, subcarrierIndex) = double(realPart) + 1i * double(imagPart);
 
            index = index + 16;
        end
    end
    % Compute the permutation array
    perm = [bitand(antenna_sel, 3) + 1, bitand(bitshift(antenna_sel, -2), 3) + 1, bitand(bitshift(antenna_sel, -4), 3) + 1];

    % Create the output structure
    ret = struct();
    ret.timestamp_low = double(timestamp_low);
    ret.bfee_count = double(bfee_count);
    ret.Nrx = double(Nrx);
    ret.Ntx = double(Ntx);
    ret.rssi_a = double(rssi_a);
    ret.rssi_b = double(rssi_b);
    ret.rssi_c = double(rssi_c);
    ret.noise = double(noise);
    ret.agc = double(agc);
    ret.perm = perm;
    ret.rate = double(fake_rate_n_flags);
    ret.csi = csi;
    ret.tv_sec = double(tv_sec);
    ret.tv_usec = double(tv_usec);
    ret.mac = char(mac);
    ret.frame_count = double(frame_count);
end
