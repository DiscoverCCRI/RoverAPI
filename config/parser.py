#!/usr/bin/env python3
import yaml
import argparse
import os
import docker
from subprocess import run
from time import sleep


def build_image(pkg: str = None):
    client = docker.DockerClient("unix:///var/run/docker.sock")
    client.images.build(path=os.getcwd(), tag="placeholder")
    pkg = pkg.strip("/")
    run(f"docker tag placeholder localhost:5000/{pkg}", shell=True)
    sleep(5)
    client.api.remove("placeholder")
    client.api.push(f'localhost:5000/{pkg}', stream=True, decode=True)
    os.remove("Dockerfile")


def write_dockerfile(info: dict = None, pkg: str = None):
    os.chdir("..")
    with open("Dockerfile", "w", encoding="utf-8") as outfile:
        outfile.write("FROM cjb873/leorover_image:1.0\n")
        outfile.write('SHELL ["/bin/bash", "-c"]\n')
        outfile.write(f"COPY {pkg} /~/{pkg}\n")
        outfile.write(f"RUN pip3 install {' '.join(info['dependencies'])}\n")
        if (info["build command"] is not None and
           not info["build command"] == "none"):
            outfile.write(f"RUN cd /~/{pkg} && {info['build command']}\n")


def load_data(filename: str = "config.yml") -> dict:

    info = None

    with open(filename, "r") as infile:
        info = yaml.safe_load(infile)

    return info


def parse_info(info: dict = None) -> {}:
    concise_info = {}

    for item in info["device"]:
        concise_info = {**concise_info, **item}

    for item in info["code"]:
        concise_info = {**concise_info, **item}

    return concise_info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", type=str, nargs=1, required=False,
                        default=["config.yml"])
    parser.add_argument("--pkg", type=str, nargs=1, required=True)

    args = parser.parse_args()

    os.chdir(args.pkg[0])
    info = load_data(args.filename[0])
    info = parse_info(info)
    write_dockerfile(info, args.pkg[0])
    build_image(args.pkg[0])


if __name__ == "__main__":
    main()
