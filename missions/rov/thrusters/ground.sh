#!/bin/bash
nsplug ground.moos targ_ground.moos --path=../../common -f
pAntler targ_ground.moos >& /dev/null &

printf "Mission started.\n"

ANSWER="0"
while [ "${ANSWER}" != "2" -a "${ANSWER}" != "q" ]; do
  printf "Hit (q) to Exit and Kill:\n"
  printf "> "
  read ANSWER
done

toutatiskill >& /dev/null &
printf "Done killing processes...\n"
rm targ_*