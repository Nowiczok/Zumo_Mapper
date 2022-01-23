classdef ZUMO < handle
    %representation of ZUMO robot
    
    properties
        raw_scan
        ip_address
        cnt = 1
    end
    
    methods
        function obj = ZUMO(ip_address)
            obj.ip_address = ip_address;
        end
        
        % gets scan from ZUMO and returns point cloud in planar coords
        function get_scan(obj)
            webread(strcat(obj.ip_address, '/start'));
            pause(0.5)
            obj.raw_scan = webread(strcat(obj.ip_address, '/get'));
            
        end
        
        % gets scan from file and returns point cloud in planar coords
        function [dists, angles] = get_scan_file(obj, path)
            strcat(path, num2str(obj.cnt), '.txt')
            scan = fileread(strcat(path, num2str(obj.cnt), '.txt'));
            obj.cnt = obj.cnt + 1;
            
            dists = [];
            angles = [];
            obj.raw_scan = scan;
            scan = strsplit(scan, ';');
            for it = 1:2:(length(scan)-1)
                dist = str2double(scan(it))/1000;
                angle = deg2rad(str2double(scan(it+1)));
                if dist > 0
                    dists = [dist, dists];
                    angles = [angle, angles];
                end
            end
        end
        
        function move(obj, button)
            switch button
                case 30
                    webread(strcat(obj.ip_address, '/forward'));
                case 29
                    webread(strcat(obj.ip_address, '/right'));
                case 28
                    webread(strcat(obj.ip_address, '/left'));
                case 31
                    webread(strcat(obj.ip_address, '/back'));
                case 115
                    webread(strcat(obj.ip_address, '/speed_up'));
                case 100
                    webread(strcat(obj.ip_address, '/speed_down'));
            end
        end
        
    end
end

