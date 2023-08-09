DEVICE=$1

case $DEVICE in

  robeert)
    echo -n "nvidia"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
