function batt_gen_out(batt_num,destination_folder,input_files,input_name)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    [file_id,file_name] = create_file_id(destination_folder,input_name);
    choice = get_run_choice();
    switch choice
        case 1
            run_from_file(file_name,input_files);
        case 2
            run_new_sim(file_id,input_files);
            produce_plot(file_name);
        otherwise
            fprintf('You have chosen to quit\n');
            return;
    end
    
end


function [file_out_id,file_name] = create_file_id(destinationfolder,input)

    out_file_original = '\EE6413_FinalProject';
    end_file = '.txt';
    file_name = strcat(out_file_original,input,end_file);
    tempfiledir = strcat(destinationfolder,'\',file_name);
    file_out_id = fopen(tempfiledir,'w'); 

end


function choice = get_run_choice()
    
    fprintf('How would you like to run.\n');
    fprintf('1)\tUse a previous Battery Model?\n');
    fprintf('2)\tRun a new Model (takes some time)\n');
    fprintf('0)\tQuit\n');
    choice = prompt('Your choice [0,1,2]:\t');

    fprintf('**************************************\n');
end


function run_from_file(file_name,input_files)

    if isfile(file_name)
        produce_plot(file_name);
    else
        fprintf('There is no model stored, creating a new one.\n');
        fprintf('**************************************\n');
        run_new_sim(file_id,input_files);
    end

end


function run_new_sim(file_id)

    out = setup();
    dim = size(out);
    for i = dim(2)
        
        fprintf(file_id,'%4.4f,',out(1,i));
        fprintf(file_id,'%4.4f,',out(2,i));
        for j = 1:dim(1) - 1
                fprintf(file_id,'%4.4f,',out(j+2,i));
        end
        fprintf(file_id,'%4.4f\n',out(j+3,i));
        
    end

end


function produce_plot(file_name)

    [t,x1,x2,x3,f] = readmatrix(file_name);
    
    figure('name','Optimum Cost');
    subplot(2,1,1)
    plot(t,f,'LineWidth',1.5);
    grid on;
    xlabel('Time index');
    ylabel('Function Cost');

    subplot(2,1,2)
    plot(t,x1,t,x2,t,x3,'LineWidth',1.5);
    grid on;
    ylim([0 1]);
    xlabel('Time index');
    ylabel('X values');
    legend('DOD','T','SOC');

end


function out = setup()

    time_max = 60;
    x(1) = socMax - socMin;     % DoD
    x(2) = 1;     % Temp
    x(3) = socMin + 0.45;    % SoC
    x(3) = enforceBCSOC(x(3));
    x_temp = x;
    x_opt = zeros(3,time_max);
    cost_opt = zeros(1,time_max);
    
    for i = 1:time_max

        if i == 1
            x_in = x;
        else
            x_in = x_opt(:,i-1);
        end

        [cost_brute,range] = brute_force(horizon,N,x_in);
        out = find_path(cost_brute,N);

        temp_compare = cost_brute(N/2,N/2,N/2);
        temp1 = cost_brute(out(:,N/2 - 1));
        temp2 = cost_brute(out(:,N/2 + 1));

        if temp1(1) < temp_compare(1)
            x_temp(:) = range(out(1,N/2 - 1),:);
            x_opt(:,i) = x_temp(:);
            cost_opt(i) = cost_brute(out(1,N/2 - 1));
        elseif temp2(1) < temp_compare(1)
            x_temp(:) = range(out(:,N/2 + 1),:);
            x_opt(:,i) = x_temp(:);
            cost_opt(i) = cost_brute(out(1,N/2 - 1));
        else
            x_opt(:,i) = x_in(:);
            cost_opt(i) = cost_opt(i-1);
        end

    end

    out = [1:time_max;cost_opt;x_opt];

end


function [cost,out] = brute_force(horizon,N,x)


    delta = horizon / N;    
    range = zeros(N,3);
    range(N/2,1) = x(1);
    range(N/2,2) = x(2);
    range(N/2,3) = x(3);
    
    for i = 1:N
        for j = 1:length(x)
            if i < (N/2)
                range((N/2) - i,j) = range((N/2) - i + 1,j) - delta;
                if j == 3
                    range((N/2) - i,j) = enforceBCSOC(range((N/2) - i,j));
                elseif j == 1
                    range((N/2) - i,j) = enforceBCDOD(range((N/2) - i,j));
                end
            
            elseif i > (N/2)
                range(i,j) = range(i - 1,j) + delta; 
                if j == 3
                    range(i,j) = enforceBCSOC(range(i,j));
                elseif j == 1
                    range(i,j) = enforceBCDOD(range(i,j));
                end
            end
        end
    end
    
    cost = policy(range,N);
    out = range;
end


