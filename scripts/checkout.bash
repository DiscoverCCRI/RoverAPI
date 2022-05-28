#!/bin/bash

filename="$1"
substr="import "


if [[ -f "packages.txt" ]]; then
    rm packages.txt 
fi

if [[ -f "packages.bash" ]]; then
    rm packages.bash
fi


while read -r line; do
    if [[ "$line" == *"$substr"* ]]; then
        pkg_name=${line#*$substr}
        echo "$pkg_name" >> packages.txt
        echo "pip3 install $pkg_name" >> packages.bash
    fi
    
done < "$filename"
