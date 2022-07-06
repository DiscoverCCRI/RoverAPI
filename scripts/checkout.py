#!/usr/bin/env python3

"""
Checkout

This script uses argv to scan any given files for imports and creates a list of
modules that need to be installed using pip called a manifest
"""


from sys import argv
from os.path import exists
from os import remove, mkdir
from subprocess import run


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
                        if '.' in line:
                            line = line[0:line.find('.')]
                        import_modules.append(line)
    return import_modules


def write_manifest(import_modules: []):
    with open('manifest', 'w', encoding='utf-8') as outfile:
        for module in import_modules:
            outfile.write(module + "\n")


def compress(arguments: []):

    mkdir('rover_experiment')
    for argument in arguments:
        run('mv ' + argument + ' rover_experiment', shell=True)
    run('mv manifest rover_experiment', shell=True)
    run('zip -rm rover_experiment.zip rover_experiment', shell=True)


def main():

    if exists('manifest'):
        remove('manifest')

    import_modules = find_modules(argv[1:])
    write_manifest(import_modules)
    compress(argv[1:])


if __name__ == '__main__':
    main()
