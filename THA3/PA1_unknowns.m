% Validation harness for the hidden PA1 datasets (cases h-k).  The logic is
% identical to `PA1_test.m` but points at the unknown files to verify the
% pipeline before submitting to the autograder.
testids = char(double('h'):double('k'));
max_diff = 0;

% Iterate over every test cases
for jj = 1:length(testids)
    testid = testids(jj);
    output_filename = sprintf("PA1-Results/pa1-%s-myoutput.txt", testid);
    
    % Read *-calbody.txt
    % Fiducial coordinates for the EM base (D), calibration object (A), and
    % probe marker (C) used to seed each frame.
    filename = sprintf("pa1-unknown-%s-calbody.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines = readlines(file_path);
    line = split(lines(1), ",");
    ND = str2double(line(1));
    NA = str2double(line(2));
    NC = str2double(line(3));
    d = zeros(3, ND);
    a = zeros(3, NA);
    c = zeros(3, NC);
    curr = 1;
    for ii = 1:ND
        curr = curr + 1;
        d(:, ii) = str2double(split(lines(curr), ","));
    end
    for ii = 1:NA
        curr = curr + 1;
        a(:, ii) = str2double(split(lines(curr), ","));
    end
    for ii = 1:NC
        curr = curr + 1;
        c(:, ii) = str2double(split(lines(curr), ","));
    end
    
    % Read *-empivot.txt
    % EM pivot calibration to recover the probe tip relative to the tracker.
    filename = sprintf("pa1-unknown-%s-empivot.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines = readlines(file_path);
    line = split(lines(1), ",");
    NG = str2double(line(1));
    Nf = str2double(line(2));
    curr = 1;
    % Define the base reference as the first frame
    G0 = zeros(3, NG);
    for ii = 1:NG
        curr = curr + 1;
        G0(:, ii) = str2double(split(lines(curr), ","));
    end
    g0 = G0-mean(G0,2);
    A = zeros(3*(Nf),6);
    B = zeros(3*(Nf),1);
    [R,p] = correspondence(g0,G0);
    A(1:3,1:3) = R;
    A(1:3,4:6) = -eye(3);
    B(1:3) = -p;
    kk = 4;
    for frame = 2:Nf
        G = zeros(3, NG);
        for ii = 1:NG
            curr = curr + 1;
            G(:, ii) = str2double(split(lines(curr), ","));
        end
        [R,p] = correspondence(g0,G);
        A(kk:kk+2,1:3) = R;
        A(kk:kk+2,4:6) = -eye(3);
        B(kk:kk+2) = -p;
        kk = kk + 3;
    end
    tG = A\B;
    
    % Read *-optpivot.txt
    % Optical pivot calibration to determine the probe tip in the camera frame.
    filename = sprintf("pa1-unknown-%s-optpivot.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines = readlines(file_path);
    line = split(lines(1), ",");
    ND = str2double(line(1));
    NH = str2double(line(2));
    Nf = str2double(line(3));
    curr = 1;
    % Define the base reference as the first frame
    D0 = zeros(3, ND);
    for ii = 1:ND
        curr = curr + 1;
        D0(:, ii) = str2double(split(lines(curr), ","));
    end
    [R, p] = correspondence(d,D0);
    FD = [R p; zeros(1,3) 1];
    H0 = zeros(3, NH);
    for ii = 1:NH
        curr = curr + 1;
        H0(:, ii) = str2double(split(lines(curr), ","));
    end
    HH0 = FD\[H0; ones(1, NH)];
    HH0 = HH0(1:3, :);
    hh0 = HH0 - mean(HH0,2);
    A = zeros(3*(Nf),6);
    B = zeros(3*(Nf),1);
    [R,p] = correspondence(hh0,HH0);
    A(1:3,1:3) = R;
    A(1:3,4:6) = -eye(3);
    B(1:3) = -p;
    kk = 4;
    for frame = 2:Nf
        D = zeros(3, ND);
        H = zeros(3, NH);
        for ii = 1:ND
            curr = curr + 1;
            D(:, ii) = str2double(split(lines(curr), ","));
        end
        for ii = 1:NH
            curr = curr + 1;
            H(:, ii) = str2double(split(lines(curr), ","));
        end
        [R, p] = correspondence(d,D);
        FD = [R p; zeros(1,3) 1];
        HH = FD\[H; ones(1, NH)];
        HH = HH(1:3, :);
        [R,p] = correspondence(hh0,HH);
        A(kk:kk+2,1:3) = R;
        A(kk:kk+2,4:6) = -eye(3);
        B(kk:kk+2) = -p;
        kk = kk + 3;
    end
    tH = A\B;
    
    % Read *-calreadings.txt
    % Run the full calibration pipeline and store the predicted C markers for
    % every frame in the format expected by the course grader.
    filename = sprintf("pa1-unknown-%s-calreadings.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines = readlines(file_path);
    line = split(lines(1), ",");
    ND = str2double(line(1));
    NA = str2double(line(2));
    NC = str2double(line(3));
    Nf = str2double(line(4));
    writelines(sprintf("%d,%d,%s",NC,Nf,output_filename),output_filename);
    % Emit the recovered EM/optical probe tips, then append each frame below.
    writematrix(round(tG(4:6)',2),output_filename,"Delimiter",",","WriteMode","append");
    writematrix(round(tH(4:6)',2),output_filename,"Delimiter",",","WriteMode","append");
    curr = 1;
    for frame = 1:Nf
        D = zeros(3, ND);
        A = zeros(3, NA);
        C = zeros(3, NC);
        for ii = 1:ND
            curr = curr + 1;
            D(:, ii) = str2double(split(lines(curr), ","));
        end
        for ii = 1:NA
            curr = curr + 1;
            A(:, ii) = str2double(split(lines(curr), ","));
        end
        for ii = 1:NC
            curr = curr + 1;
            C(:, ii) = str2double(split(lines(curr), ","));
        end
        [R, p] = correspondence(d,D);
        FD = [R p; zeros(1,3) 1];
        [R, p] = correspondence(a,A);
        FA = [R p; zeros(1,3) 1];
        C_expected = FD\FA*[c; ones(1,NC)];
        writematrix(round(C_expected(1:3,:)',2),output_filename,"Delimiter",",","WriteMode","append")
    end
end