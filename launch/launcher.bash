echo "Start robot"
gnome-terminal -x sh -c "roslaunch gpd_interface start.launch;"

sleep 20

echo "Start nbv"
gnome-terminal -x sh -c "roslaunch gpd_interface nbv.launch;"

sleep 10

echo "Start gpd"
gnome-terminal -x sh -c "roslaunch gpd_interface gpd.launch;"
