#!/bin/bash
if [[ $# -eq 0 ]] ; then
    echo 'One arument expected: command to launch'
    exit 1
fi
prefix="[$1]:"

echo $prefix "starting $1"

# Run the command in loop unless the exit code is 0 (aka. success)
# the  &> >(xargs -I {} echo $prefix {})  allow to add the prefix to every line outputed on any stream
until $1 &> >(xargs -n 1 -I "{}" echo $prefix "{}"); do
    echo $prefix "$1 crashed : Restarting ..."
done 2> /dev/null # Do not print crash errors as we already print it ourself

echo $prefix "$1 closed properly. exit."
