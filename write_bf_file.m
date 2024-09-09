%WRITE_BF_FILE Writes back to a file of beamforming feedback logs, by
% (c) 2024 Brendan Dijkstra <b.j.dijkstra.2@student.rug.nl>
%
function write_bf_file(filepath, csi_data)
    % Open the file for binary writing
    fileID = fopen(filepath, 'wb');
    
    % Check if file was opened successfully
    if fileID == -1
        error('Unable to open file for writing.');
    end
    
    % Check input
    if ~isstruct(csi_data)
        error('Input should be a struct.');
    end

    % Process each entry in the cell array
    for idx = 1:length(csi_data)
        % Skip empty entries
        if isempty(csi_data(idx).time) || csi_data(idx).time==0
            warning('Entry at index %d is empty.', idx);
            continue; % Skip this iteration if the cell is empty
            %break % probably no more good entries after this.
        end
        
        % Calculate hex data and write to file
        binaryData = write_bfee_local(csi_data(idx)); % Convert struct to binary data
        fwrite(fileID, binaryData, 'uint8'); % Write binary data to file
    end

    % Close the file
    fclose(fileID);
end

function binaryData = write_bfee_local(struct)

    % troubleshoot using dec2hex()
    
    % Start properly defining all data from the struct
    time = typecast(uint32(struct.time), 'uint8'); % UINT32 - Little Endian (DCBA)
    bfee_count = typecast(uint16(struct.bfee_count), 'uint8'); % INT16 - Little Endian (BA)
    Nrx = uint8(struct.Nrx);
    Ntx = uint8(struct.Ntx);
    rssi_a = uint8(struct.rssi_a);
    rssi_b = uint8(struct.rssi_b);
    rssi_c = uint8(struct.rssi_c);
    noise = uint8(struct.noise);
    agc = uint8(struct.agc);
    antenna_sel_array = uint8(struct.perm);
    fake_rate_n_flags = typecast(uint16(struct.rate), 'uint8');
    tv_sec = typecast(uint32(struct.tv_sec), 'uint8'); % UINT32 - Little Endian (DCBA)
    tv_usec = typecast(uint32(struct.tv_usec), 'uint8'); % UINT32 - Little Endian (DCBA)
    mac = char(struct.mac);
    frame_count = typecast(uint32(struct.frame_count), 'uint8'); % UINT32 - Little Endian (DCBA)

    % Encode field_len and 187 code
    csi_len = round((30 * (double(Nrx) * double(Ntx) * 8 * 2 + 3) + 7) / 8,0);
    % header 41 bytes + csi length
    struct_len = 41 + csi_len;
    
    % Initialize binary data array
    binaryData = zeros(1, struct_len, 'uint8');
    
    % Standard data in first 3 bytes
    binaryData(1:2) = typecast(swapbytes(uint16(struct_len-2)), 'uint8'); % INT16 - Big Endian (AB)
    binaryData(3) = uint8(187); % Constant code, binary digits "BB"
    
    % Assuming perm is an array with your permuted values
    perms = [antenna_sel_array(1) - 1, antenna_sel_array(2) - 1, antenna_sel_array(3) - 1];
    
    % Reconstruct antenna_sel from perm values
    antenna_sel = bitshift(perms(3), 4) + bitshift(perms(2), 2) + perms(1);


    % Encode time
    binaryData(4:7) = time;
    binaryData(8:9) = bfee_count;
    binaryData(10:11) = uint8(0); %undefined
    binaryData(12) = Nrx;
    binaryData(13) = Ntx;
    binaryData(14) = rssi_a;
    binaryData(15) = rssi_b;
    binaryData(16) = rssi_c;
    binaryData(17) = noise; % not captured properly?
    binaryData(18) = agc;
    binaryData(19) = antenna_sel;
    binaryData(20:21) = typecast(uint16(csi_len),'uint8');
    binaryData(22:23) = fake_rate_n_flags;
    binaryData(24:27) = tv_sec;
    binaryData(28:31) = tv_usec;
    binaryData(32:37) = hex2dec(transpose(reshape(mac,2,[])));
    binaryData(38:41) = frame_count;
    binaryData(42:end) = write_csi_to_binary(struct.csi, antenna_sel_array);
end


function binaryCSI = write_csi_to_binary(csi,perm)
    [Ntx,Nrx,numSubcarriers]=size(csi);

    %might be incorrect!
    binaryCSI = zeros(1, ceil((numSubcarriers * (Ntx * Nrx * 16 + 3))/8), 'uint8'); 
    bit_index=0;

    % First and last byte positions are only made up by 1 number
    for subcarrierIndex = 1:numSubcarriers
        bit_index = bit_index + 3; % Skip the first 3 bytes as per the original logic
        bit_remainder = mod(bit_index,8);
        for txIndex = 1:Ntx
            % check on the permutation (perm) array related to the received 
            % antenna configuration and possibly adjusts the CSI matrix 
            % based on this permutation
            if Nrx==1
                Nrx_perm_fixed=1;
            elseif sum(perm) ~= sum(1:Nrx) % matrix does not contain default values
                warning('CSI with invalid perm');
                Nrx_perm_fixed = 1:Nrx;
            else
                Nrx_perm_fixed = perm;
            end
            % Now loop over Rx values based on the permutation of antenna array
            for rxIndex = Nrx_perm_fixed
                realPart = real(csi(txIndex, rxIndex, subcarrierIndex));
                imagPart = imag(csi(txIndex, rxIndex, subcarrierIndex));
                
                % Scale and round the real and imaginary parts to fit into 8 bits
                realPart = typecast(int8(realPart), 'uint8');
                imagPart = typecast(int8(imagPart), 'uint8');
                
                real_shift_1 = bitshift(realPart, bit_remainder,"uint8");
                real_shift_2 = bitshift(realPart, bit_remainder-8,"uint8");
                imag_shift_1 = bitshift(imagPart, bit_remainder,"uint8");
                imag_shift_2 = bitshift(imagPart, bit_remainder-8,"uint8");
                
                array_index = floor(bit_index / 8) + 1;
                binaryCSI(array_index) = bitor(binaryCSI(array_index),real_shift_1);
                binaryCSI(array_index+1) = bitor(binaryCSI(array_index+1),real_shift_2);
                binaryCSI(array_index+1) = bitor(binaryCSI(array_index+1),imag_shift_1);
                binaryCSI(array_index+2) = bitor(binaryCSI(array_index+2),imag_shift_2);

                bit_index=bit_index+16;
            end
        end
    end
end
