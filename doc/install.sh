echo 'Spark driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/uarm-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/spark-usb-serial.rules /etc/udev/rules.d/
sudo $BASEPATH/rules/eai_gx.sh
sudo udevadm trigger

echo 'Spark driver is installed'

