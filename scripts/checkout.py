#!/usr/bin/env python3

"""
Checkout

This script uses the importlib module to install any necessary python modules
on a docker container for the user. Any relevant .py files should be passed as
command line arguments for the script to check for import statements. If the
module cannot be found by pip, the name of the module is added to the pip.err
file
"""


from importlib.util import find_spec
from subprocess import run
from sys import argv, modules
from datetime import datetime
from os.path import exists
from os import remove


def find_modules(arguments: []) -> []:
    import_modules = []

    # open each .py that is passed as a command line argument
    for argument in arguments:
        with open(argument, 'r', encoding='utf-8') as infile:
            # check each line for relevant import statements
            # if there is one, strip it down to the name for pip
            for line in infile:
                line = line.strip()
                if not ('rover_api' in line):
                    if 'from' in line:
                        line = line[5:]
                        if '.' in line:
                            line = line[0:line.find('.')]
                        else:
                            line = line[0:line.find(' ')]
                        import_modules.append(line)
                    elif 'import' in line:
                        line = line[7:]
                        line = line[:line.find(' ')]
                        if '.' in line:
                            line = line[0:line.find('.')]
                        import_modules.append(line)
                else:
                    print("RoverAPI is already installed")
    return modules


def install_modules(import_modules: []):
    for module in import_modules:
        # check if the module is in the python path
        if module in modules:
            print(module + ' is in sys.modules')
        # otherwise, check if the module has already been installed
        elif find_spec(module) is not None:
            print(module + ' is already installed')
        # otherwise, try to install the module with pip
        else:
            try:
                run('pip3 install ' + module, shell=True, check=True)
            except Exception:
                # write the name and the time of the module to pip.err
                with open('pip.err', 'a') as outfile:
                    time = datetime.now()
                    time_str = time.strftime('%d-%m-%Y %H:%M:%S')
                    out_str = time_str + ' Could not install ' + module + '.\n'
                    outfile.write(out_str)


def main():
    if exists('pip.err'):
        remove('pip.err')

    import_modules = find_modules(argv[1:])
    install_modules(import_modules)


if __name__ == '__main__':
    main()
