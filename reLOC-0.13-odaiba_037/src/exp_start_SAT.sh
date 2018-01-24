SIZE=$1
OBSTACLES=$2
ROBOTS=$3

./exp_gridgen.sh $SIZE $OBSTACLES $ROBOTS

#./exp_gridsol.sh $SIZE Hmatching &
#./exp_gridsol.sh $SIZE Hsimplicial &
#./exp_gridsol.sh $SIZE Hdifferential &
#./exp_gridsol.sh $SIZE Hdirect &
#./exp_gridsol.sh $SIZE Hadvanced &
#./exp_gridsol.sh $SIZE plural &
#./exp_gridsol.sh $SIZE singular &
#./exp_gridsol.sh $SIZE plural2 &
#./exp_gridsol_uni.sh $SIZE plural &
./exp_gridsol_mdd.sh $SIZE mdd $ROBOTS &
#./exp_gridsol_mdd.sh $SIZE nomdd $ROBOTS &
#./exp_gridsol_mdd.sh $SIZE rxbmdd $ROBOTS &
./exp_gridsol_mdd.sh $SIZE bmdd $ROBOTS &
./exp_gridsol_mdd.sh $SIZE bcmdd $ROBOTS &