#!/bin/bash

cd "${0%/*}"
docker build -t robowflex/indigo -f Dockerfile `git rev-parse --show-toplevel`


docker run -t --network=host --name indigo_test -d robowflex/indigo
docker exec indigo_test /bin/bash -c "source ros_entrypoint.sh; nohup roscore &> /dev/null &"
docker exec indigo_test /bin/bash -c "source ros_entrypoint.sh; cd /ws/; catkin run_tests robowflex_library --no-deps; catkin_test_results"
value=$?
docker stop indigo_test
docker rm indigo_test

echo $value
