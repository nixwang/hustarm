#!/bin/bash

$(dirname $0)/interceptty -q -s 'ispeed $2 ospeed $2' -l $1 $3
