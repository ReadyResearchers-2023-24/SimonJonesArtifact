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
  --num-worlds=<int>     Number of worlds to generate [default: 10]
  --worlds-dir=<path>    Directory where generated worlds will be saved
                         [default: pcg/models/].
  --models-dir=<path>    Directory where generated models will be saved
                         [default: pcg/resources/worlds].

'''

if __name__ == "__main__":
    opts = docopt(USAGE)

    worlds_dir = opts['--worlds-dir'] or None
    models_dir = opts['--models-dir'] or None
    num_worlds = int(opts['--num-worlds'])

    # using +2 because minimum number of rectangles is 2
    # when using plural method
    for i in [number + 2 for number in range(num_worlds)]:
        generate_room(
            n_rectangles=i,
            filename=f"{i}-rectangles-walls",
            worlds_dir_path=worlds_dir,
            models_dir_path=models_dir
        )
