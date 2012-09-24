#!/bin/bash
run_exp()
{
    #set og_index 
    sed -i -e "s/og_index\ [0-9][0-9]/og_index\ $1/" ./option.cfg

    for(( i=1;i<=$2;i++))
    do
        stage -g -c ./world/dars2012.world
        sleep 1
    done
}

#run_exp 0 10
#run_exp 1 10
#run_exp 2 10
#run_exp 3 10
#run_exp 4 10 
#run_exp 5 10
#run_exp 6 10
#run_exp 7 10
#run_exp 8 10
#run_exp 9 10
#run_exp 10 10
run_exp 11 10
run_exp 12 10
run_exp 13 10
run_exp 14 10
run_exp 15 10
run_exp 16 10
run_exp 17 10
run_exp 18 10
run_exp 19 10
run_exp 20 10
run_exp 21 10
