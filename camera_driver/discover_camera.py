#!/usr/bin/env python3

from subprocess import check_output
from re import findall

data = check_output("rostopic echo -n 1 /camera/image_raw/compressed  | grep data", shell=True)
data = data.lstrip("data: [")
data = data.rstrip()
data = data.rstrip("]")


first_item = int(data[0:3])
length_of_string = len(data)
last_item = int(data[length_of_string - 3:length_of_string])

found = findall("(?<=,)(.*?)(?=,)", data)
integer_map = map(int, found)
integer_list = list(integer_map)
integer_list.insert(0, first_item)
integer_list.append(last_item)
print(integer_list)
