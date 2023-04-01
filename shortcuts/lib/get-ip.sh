LOCATION=$1

case $LOCATION in

  home)
    echo -n "192.168.0.196"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid location name: ${LOCATION}"
    ;;
esac
