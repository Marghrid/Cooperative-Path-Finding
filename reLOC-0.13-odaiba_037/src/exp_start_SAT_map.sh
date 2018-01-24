MAP=$1
ROBOTS=$2
DISTANCES=$3

./exp_mapgen.sh $MAP $ROBOTS $DISTANCES

#./exp_mapsol_mdd.sh mdd $MAP $ROBOTS $DISTANCES &
./exp_mapsol_mdd.sh admdd++ $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh bmdd $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh bcmdd $MAP $ROBOTS $DISTANCES &