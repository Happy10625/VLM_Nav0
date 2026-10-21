"""Ranger single-robot entry; shared executor, no bridge or coordinator."""
from .go2_check import main as shared_main


def main(argv=None):
    return shared_main(argv,robot_id='ranger')


if __name__=='__main__':
    raise SystemExit(main())
