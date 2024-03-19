import os

from docopt import docopt
from sys import stdout
from generate import generate_room

USAGE = '''

pcg - Script for procedural world generation in gazebo.

Usage:
  pcg [--num-worlds=<int>] [--worlds-dir=<path>] [--models-dir=<path>]

Options:
  -h, --help             Show this screen.
  --num-worlds=<int>     Number of worlds to generate [default is 10]
  --worlds-dir=<path>    Directory where generated worlds will be saved [default is pcg/models/].
  --models-dir=<path>    Directory where generated models will be saved [default is pcg/resources/worlds].

'''

if __name__ == "__main__":
    opts = docopt(USAGE)

    worlds_dir = opts['--worlds-dir'] or None
    models_dir = opts['--models-dir'] or None
    num_worlds = opts['--num-worlds'] or 10

    for i in [8, 9, 10, 11]:
        print(f"({i} rectangles) starting")
        generate_room(
            filename=f"{i}-rectangles-walls",
            n_rectangles=i,
            worlds_dir_path=worlds_dir,
            models_dir_path=models_dir
        )
        print(f"({i} rectangles) finished")
    exit(0)

    # using +2 because minimum number of rectangles is 2
    # when using plural method
    for i in [number + 2 for number in range(num_worlds)]:
        generate_room(
            filename=f"{i}-rectangles-walls",
            n_rectangles=i,
            worlds_dir_path=worlds_dir,
            models_dir_path=models_dir
        )
