#!/bin/bash

PFILE=p1

if [ "$1" != "" ]; then
  PFILE="$1"
fi

echo "Planning for $PFILE.pddl -> generating $PFILE.plan"

../bin/Contingent-FF -I -p ./ -o domain.pddl -f $PFILE.pddl > $PFILE.plan

