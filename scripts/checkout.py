#!/usr/bin/env python3

from importlib.util import find_spec
from subprocess import run
import sys


def find_modules(arguments: []) -> []:
    modules = []

    for argument in arguments:
        with open(argument, 'r', encoding='utf-8') as infile:
            for line in infile:
                line = line.strip()
                if 'from' in line:
                    line = line[5:]
                    if '.' in line:
                        line = line[0:line.find('.')]
                    else:
                        line = line[0:line.find(' ')]
                    modules.append(line)
                elif 'import' in line:
                    line = line[7:]
                    if '.' in line:
                        line = line[0:line.find('.')]
                    modules.append(line)
    return modules


def install_modules(modules: []):
    for module in modules:
        if module in sys.modules:
            print(module + ' is in sys.modules')
        elif find_spec(module) is not None:
            print(module + ' is already installed')
        else:
            try:
                run('pip3 install ' + module, shell=True, check=True)
            except Exception:
                print('Could not install ' + module + '. Try manual install.')


def main():
    modules = find_modules(sys.argv[1:])
    install_modules(modules)


if __name__ == '__main__':
    main()
