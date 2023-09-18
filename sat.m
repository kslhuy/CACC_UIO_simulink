function y = sat(x, max_val ,min_val )
    % Ensure x is within the specified bounds
    y = max(min(x, max_val), min_val);
%     y = x/(abs(x)+d);
end


