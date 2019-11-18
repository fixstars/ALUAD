#!/bin/bash


while [[ "$#" -gt 0 ]]; do case $1 in
  -r|--remove-files) remove=1; shift;;
  -v|--verbose) verbose=1;;
  *) echo "Unknown parameter passed: $1"; exit 1;;
esac; shift; done

cat ../../data/*/*.csv > ../../data/all.csv

if [ "$verbose" = "1" ]
then
  read lines words chars <<< $(wc ../../data/all.csv)
  echo "Done!"
  echo "Total number of samples: $lines"
fi

if [ "$remove" = "1" ]
then
  rm ../../data/*/*.csv
fi
