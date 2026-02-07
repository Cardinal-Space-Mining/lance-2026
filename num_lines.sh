#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

wc -l \
    $SCRIPTPATH/{cardinal-perception,csm-metrics,csm-sim,lance,launch-utils,net-adapter}/{*/*,*/*/*,*/*/*/*}.{cpp,hpp,py} \
    $SCRIPTPATH/{multiscan-driver,phoenix-driver}/{*/*,include/*/*}.{cpp,hpp,py}
