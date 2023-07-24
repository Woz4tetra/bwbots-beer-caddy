DEVICE=$1

case $DEVICE in

  robeert)
    echo -n "192.168.0.196"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
