#!/bin/bash

#$# -> number of arguments
#$1 -> first argument

if [ "$#" -eq 1 ]; 
then
   if [ -f "$1" ]; then 
      #echo $1;
      echo -e "\n flashing $1\n";
      st-flash write $1 0x08000000
   else 
      echo -e "\nfile not found\n"; exit 1; fi
else
   echo -e "\n flashing build/LishuiFOC_01.bin\n";
   st-flash write build/LishuiFOC_01.bin 0x08000000
fi

