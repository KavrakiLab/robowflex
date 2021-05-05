#!/bin/bash

function check-docker-tests()
{
    local container=$1
    docker run -t --network=host --name ${container}_test -d robowflex/${container}
    docker exec ${container}_test /bin/bash -c "source ros_entrypoint.sh; nohup roscore &> /dev/null &"
    docker exec ${container}_test /bin/bash -c "source ros_entrypoint.sh; cd /ws/; catkin run_tests robowflex_library --no-deps --limit-status-rate 0.001 --no-notify; catkin_test_results"
    local result=$?
    docker stop ${container}_test
    docker rm ${container}_test
    echo "$result"
}
