classdef scan_manager < handle
    
    properties
        slam_alg
        scans = []
        super_scan
        
        res
        max_range
        lc_thr
        lc_sr_rad
        aux
        x_points_den = 5
        x_points
        y_points_den = 5
        y_points
        phi_points
        phi_points_den = 10
    end
    
    methods
        function obj = scan_manager(resolution, max_range, lc_threshold, lc_search_rad)
            obj.res = resolution;
            obj.max_range = max_range;
            obj.lc_thr = lc_threshold;
            obj.lc_sr_rad = lc_search_rad;
            obj.reset_scan_handle(resolution, max_range, lc_threshold, lc_search_rad);
        end
        
        function reset_scan_handle(obj, resolution, max_range, lc_threshold, lc_search_rad)
            obj.slam_alg = lidarSLAM(resolution, max_range);
            obj.slam_alg.LoopClosureThreshold = lc_threshold;  
            obj.slam_alg.LoopClosureSearchRadius = lc_search_rad;
        end
        
        function add_scan(obj, raw_scan)
            obj.scans = [obj.scans, string(raw_scan)];  % append scan 
            obj.new_scan(raw_scan);
        end
        
        function new_scan(obj, raw_scan)
            [dists, angles] = obj.process_scan(raw_scan);
            scan = lidarScan(dists, angles);
            addScan(obj.slam_alg, scan);
        end
        
        function [dists, angles] = process_scan(obj, raw_scan)
            dists = [];
            angles = [];
            scan = strsplit(string(raw_scan), ';');
            for it = 1:2:(length(scan)-1)
                dist = str2double(scan(it))/1000;
                angle = deg2rad(360-str2double(scan(it+1)));
                if dist > 0
                    dists = [dist, dists];
                    angles = [angle, angles];
                end
            end
        end
        
        function remove_scan(obj, num)
            obj.scans = obj.scans([1:(num-1), (num+1):end]);
            obj.remake_map() 
        end
        
        function remake_map(obj)
            obj.reset_scan_handle(obj.res, obj.max_range, obj.lc_thr, obj.lc_sr_rad);
             for scan = obj.scans
                 obj.new_scan(scan);  
             end
        end
        
        function pop_scan(obj)
            length(obj.scans)
            obj.remove_scan(length(obj.scans));
        end
        
        function pose = get_pos(obj, raw_scan)
            [dists, angles] = obj.process_scan(raw_scan);
            scan = lidarScan(dists, angles);
            obj.aux = scan;
            prev_score = 0;
            for x = obj.x_points
               for y = obj.y_points
                   for phi = obj.phi_points
                       [pose_aux, stats] = matchScans(scan, obj.super_scan, "InitialPose", [x, y, phi], "MaxIterations", 400, "SolverAlgorithm", "trust-region", "CellSize", 1, "ScoreTolerance", 1e-6);
                       if stats.Score > prev_score
                           prev_score = stats.Score;
                           pose = pose_aux;
                       end
                   end
               end
            end
        end
        
        function create_super_scan(obj)
            [proc_scans, optimizedPoses] = scansAndPoses(obj.slam_alg);
            super_dists = [];
            super_angles = [];
            for i = 1:(length(proc_scans))
                scan = proc_scans(i);
                optimizedPoses(1, :)
                tran_scan = transformScan(scan{1}, optimizedPoses(i, :));
                super_dists = [super_dists, tran_scan.Ranges(:)'];
                super_angles = [super_angles, tran_scan.Angles(:)'];
            end
            
            [x, y] = pol2cart(super_angles, super_dists);
            max_x = max(x);
            min_x = min(x);
            pp_x = max_x - min_x;
            max_y = max(y);
            min_y = min(y);
            pp_y = max_y - min_y;
            obj.x_points = min_x:(pp_x/obj.x_points_den):max_x;
            obj.y_points = min_y:(pp_y/obj.y_points_den):max_y;
            obj.phi_points = 0:(2*pi/10):(2*pi);
            
            scan_matrix = [super_dists(:), super_angles(:)];
            scan_matrix = sortrows(scan_matrix, 2);
            scan_matrix(:, 2);
            obj.super_scan = lidarScan(scan_matrix(:, 1), scan_matrix(:, 2)); % dists, angles
        end
        
    end
end

