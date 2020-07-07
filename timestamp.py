import time


def doing():
    t = time.time() % 0.05 * 1000
    print(t)
    time.sleep(0.05)


if __name__ == '__main__':
    while True:
        doing()