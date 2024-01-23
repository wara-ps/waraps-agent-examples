import argparse
import os
from threading import Thread


def task(args: str, index: int) -> None:
    os.system(f"python -u main.py {args}_{index}")


def make_str_from_args(args) -> str:
    if args.name:
        my_str = f"-u {args.units} -n {args.name}"
    else:
        my_str = f"-u {args.units}"

    return my_str


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name", help="Give the agents a name")
    parser.add_argument("-u", "--units", help="Input how many agents to create", required=True)
    args = parser.parse_args()

    threads = []
    args_str = make_str_from_args(args)
    i: int = 1
    for _ in range(int(args.units)):
        new_thread = Thread(target=task, args=(args_str, i))
        new_thread.start()
        threads.append(new_thread)
        i += 1

    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
