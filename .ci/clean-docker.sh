#!/bin/bash

O1=$(docker ps --filter=status=exited --filter=status=created -q)
if [[ ! -z $O1 ]]
then
    docker rm $O1
fi

O2=$(docker images -a --filter=dangling=true -q)
if [[ ! -z $O2 ]]
then
    docker rmi $O2
fi
