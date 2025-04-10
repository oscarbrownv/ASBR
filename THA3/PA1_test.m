testids = char(double('c'):double('c'));

% Iterate over every test cases
for jj = 1:length(testids)
    testid = testids(jj);
    output_filename = sprintf("PA1-Results/pa1-%s-myoutput.txt", testid);
    
    % Read *-calbody.txt
    filename = sprintf("pa1-debug-%s-calbody.txt", testid);
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
    filename = sprintf("pa1-debug-%s-empivot.txt", testid);
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
    A = zeros(3*(Nf-1),6);
    B = zeros(3*(Nf-1),1);
    kk = 1;
    for frame = 2:Nf
        G = zeros(3, NG);
        for ii = 1:NG
            curr = curr + 1;
            G(:, ii) = str2double(split(lines(curr), ","));
        end
        [R,p] = correspondence(G0,G);
        A(kk:kk+2,1:3) = R;
        A(kk:kk+2,4:6) = -eye(3);
        B(kk:kk+2,1) = -p;
        kk = kk + 3;
    end
    tG = A\B;
    
    % Read *-optpivot.txt
    filename = sprintf("pa1-debug-%s-optpivot.txt", testid);
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
    [R, p] = correspondence(d,D);
    FD = [R p; zeros(1,3) 1];
    H0 = zeros(3, NH);
    for ii = 1:NH
        curr = curr + 1;
        H0(:, ii) = str2double(split(lines(curr), ","));
    end
    HH0 = FD\[H0; ones(1, NH)];
    HH0 = HH0(1:3, :);
    A = zeros(3*(Nf-1),6);
    B = zeros(3*(Nf-1),1);
    kk = 1;
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
        [R,p] = correspondence(HH0,HH);
        A(kk:kk+2,1:3) = R;
        A(kk:kk+2,4:6) = -eye(3);
        B(kk:kk+2,1) = -p;
        kk = kk + 3;
    end
    tH = A\B;
    
    % Read *-calreadings.txt
    filename = sprintf("pa1-debug-%s-calreadings.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines = readlines(file_path);
    line = split(lines(1), ",");
    ND = str2double(line(1));
    NA = str2double(line(2));
    NC = str2double(line(3));
    Nf = str2double(line(4));
    writelines(sprintf("%d,%d,%s",NC,Nf,output_filename),output_filename);
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

    % Check correctness and read
    filename = sprintf("pa1-debug-%s-output1.txt", testid);
    file_path = fullfile("HW3-PA1", filename);
    lines_debug = readlines(file_path);
    lines_mine = readlines(output_filename);
    line_debug = split(lines_debug(1), ",");
    line_mine = split(lines_mine(1), ",");
    assert(str2double(line_debug(1)) == str2double(line_mine(1)));
    assert(str2double(line_debug(2)) == str2double(line_mine(2)));
    NC = str2double(line_debug(1));
    Nf = str2double(line_debug(2));
    tol = 1.1e-2;
    curr = 1;
    for frame = 1:Nf
        curr = curr + 1;
        data_debug = str2double(split(lines_debug(curr), ","));
        data_mine = str2double(split(lines_mine(curr), ","));
        % assert(norm(data_debug-data_mine) <= tol, sprintf("Fail @ line #%d", curr))
    end
end