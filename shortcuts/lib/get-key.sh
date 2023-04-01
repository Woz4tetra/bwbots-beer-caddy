DEVICE=$1

case $DEVICE in

  robeert)
    echo -n "~/.ssh/robeert"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
