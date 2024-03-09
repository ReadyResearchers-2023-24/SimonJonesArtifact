import os

from docopt import docopt
from sys import stdout
from .generate import generate_room

USAGE = '''

pcg - Script for procedural world generation in gazebo.

Usage:
  pcg [--world-path=<path>] [--model-path=<path>] [--num-worlds=<int>]

Options:
  -h, --help             Show this screen.
  --num-worlds           Number of worlds to generate [default: 10]
  --world-path=<path>    Folder where generated worlds will be saved
                         [default: ~/.gazebo/worlds].
  --model-path=<path>    Folder where generated models will be saved
                         [default: ~/.gazebo/models].

'''

def pcg():
    opts = docopt(USAGE)

    world_path = opts['--world-path']
    model_path = opts['--model-path']
    num_worlds = int(opts['--num-worlds'])

    model_base_path = os.path.expanduser(opts['--model-path'])
    world_base_path = os.path.expanduser(opts['--world-path'])

    raise Exception("FIXME: not implemented.")

    # using +2 because minimum number of rectangles is 2
    # when using plural method
    for i in [number + 2 for number in range(num_worlds)]:
        generate_room(n_rectangles=i, filename=f"{i}-rectangles-walls")
