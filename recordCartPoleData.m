function idData = recordCartPoleData(duration)
    arguments
        duration (1,1) {mustBeNumeric, mustBePositive}
    end

    %–– Initialize ROS 2 node and allow it to settle
    node = ros2node("id_node");
    pause(1);

    %–– Determine the pole joint index (ROS 2 uses lowercase fields)
    jsSub    = ros2subscriber(node, "/joint_states", "sensor_msgs/JointState");
    jointMsg = receive(jsSub, 5);
    poleIdx  = find(strcmp(jointMsg.name, "cart_to_pole"), 1);
    if isempty(poleIdx)
        error("'cart_to_pole' not found. Available: %s", strjoin(jointMsg.name, ", "));
    end

    %–– Preallocate storage
    data.u  = []; data.tu = [];
    data.y  = []; data.ty = [];

    %–– Stamp start time as numeric seconds
    t0msg = ros2time(node, "now");
    t0    = double(t0msg.sec) + double(t0msg.nanosec)*1e-9;

    %–– Subscribers with callbacks (single-argument signature)
    ros2subscriber(node, "/effort_controllers/commands", ...
                   "std_msgs/Float64MultiArray", @storeU);
    ros2subscriber(node, "/joint_states", ...
                   "sensor_msgs/JointState",       @storeY);

    %–– Loop for duration seconds at 100 Hz using ROS2 rate
    r = ros2rate(node, 100);
    while true
        %–– Check elapsed time by converting Time struct to seconds
        tNmsg = ros2time(node, "now");
        tNow  = double(tNmsg.sec) + double(tNmsg.nanosec)*1e-9;
        if tNow - t0 >= duration
            break;
        end
        waitfor(r);
    end

    %–– Build synchronized iddata
    tCommon = sort(unique([data.tu; data.ty]));
    uComm   = interp1(data.tu, data.u, tCommon, 'previous', 'extrap');
    yComm   = interp1(data.ty, data.y, tCommon, 'previous', 'extrap');
    Ts      = mean(diff(tCommon));
    idData  = iddata(yComm, uComm, Ts, 'TimeUnit','s','InterSample','zoh');

    %–– Callback subfunctions
    function storeU(msg)
        tmsg = ros2time(node, "now");
        t    = double(tmsg.sec) + double(tmsg.nanosec)*1e-9;
        data.u(end+1,1)  = double(msg.Data(1));
        data.tu(end+1,1) = t;
    end

    function storeY(msg)
        tmsg = ros2time(node, "now");
        t    = double(tmsg.sec) + double(tmsg.nanosec)*1e-9;
        data.y(end+1,1)  = msg.Position(poleIdx);
        data.ty(end+1,1) = t;
    end
end