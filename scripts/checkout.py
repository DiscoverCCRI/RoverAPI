#!/usr/bin/env python3

from importlib.util import find_spec
from subprocess import run
from sys import argv, modules
from datetime import datetime
from os.path import exists
from os import remove


def find_modules(arguments: []) -> []:
    import_modules = []

    for argument in arguments:
        with open(argument, 'r', encoding='utf-8') as infile:
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
        if module in modules:
            print(module + ' is in sys.modules')
        elif find_spec(module) is not None:
            print(module + ' is already installed')
        else:
            try:
                run('pip3 install ' + module, shell=True, check=True)
            except Exception:
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
