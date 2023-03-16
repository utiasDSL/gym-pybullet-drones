while :
do
  kara=$(qstat | grep 21D30114)
  if [ -z "$kara" ]; then
  	date
    echo "***********there is no job************"
  else
  	date
    echo $kara
    #echo "ff"
  fi

  kara=$(t3-user-info disk home)
  echo $kara  
  sleep 10

  kara=$(t3-user-info group point)
  echo $kara  
  sleep 10


done

