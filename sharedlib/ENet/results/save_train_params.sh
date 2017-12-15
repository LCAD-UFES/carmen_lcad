#!/bin/bash
USAGE="Usage: $0 <solver_prototxt_file>"
[ $# -ne 1 ] && echo "$USAGE" && exit
! [ -f $1 ] && echo "$1: No such file" && echo "$USAGE" && exit
PWD_ORIGINAL=$PWD
SOLVER="$1"
[ "${SOLVER::2}" == "./" ] && SOLVER=${SOLVER:2}
while [ "${SOLVER::3}" == "../" ]
do
   cd ..
   SOLVER=${SOLVER:3}
done
SOLVER=$PWD/$SOLVER
cd $PWD_ORIGINAL
TRAIN=`grep -e "net:" $SOLVER | cut -d\" -f 2`
SOURCE=`grep -e "source:" $TRAIN | cut -d\" -f 2`
EXAMPLES=`wc -l $SOURCE | cut -d' ' -f 1`
BATCH=`grep -e "batch_size:" $TRAIN | cut -d: -f 2 | cut -d# -f 1 | awk '{$1=$1}1'`
MAX_ITER=`grep -e "max_iter:" $SOLVER | cut -d: -f 2 | cut -d# -f 1 | awk '{$1=$1}1'`
EPOCHS=$(( $MAX_ITER * $BATCH / $EXAMPLES ))
ITERS_REM=$(( $MAX_ITER * $BATCH % $EXAMPLES ))
EPOCHS_FRAC=$(( 1000 * $MAX_ITER * $BATCH / $EXAMPLES + 2000 * $MAX_ITER * $BATCH / $EXAMPLES % 2 ))
[ $ITERS_REM -ne 0 ] && EPOCHS=`echo $EPOCHS_FRAC | sed s/$EPOCHS/$EPOCHS./`
TS=`date -Iminutes | sed y/-T:/___/ | cut -d_ -f 1,2,3,4,5`
PARAM=`echo $CARMEN_HOME/sharedlib/ENet/results/parameters_$TS.txt | sed s:$PWD/::`
echo > $PARAM
echo [$SOURCE]: >> $PARAM
echo -e "# dataset_train_size = $EXAMPLES\t# Numero total de exemplos de treinamento" >> $PARAM
echo -e "# epochs = $EPOCHS\t\t# Numero de epocas de treinamento" >> $PARAM
echo >> $PARAM
echo [$TRAIN]: >> $PARAM
grep -e "source:" -e "batch_size:" $TRAIN >> $PARAM
echo >> $PARAM
echo [$SOLVER]: >> $PARAM
cat $SOLVER >> $PARAM
echo File $PARAM saved.
