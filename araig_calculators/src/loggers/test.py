import os

def check_file(folder):
    try:
        files = os.listdir(folder)

        print(files)
        print(type(files))
    except OSError:
        pass

if __name__ == "__main__":
    files = ['param_calculators.yaml', 'param_runner.yaml', 'test1_braking_2021-04-15-22-38-59.bag', 'debug_braking_time_node.yaml', 'param_interpreters.yaml', 'robot_speed.csv']
    types = ['param', 'debug', 'result']
    flag = 0
    l = 0
    for t in types:
        if any(t in f for f in files):
            l += 1
            print(t, True)
    print(l)
            # return False
    # return True
    print(True)
    deduplicated = ['facebook.com','google.com','en.wikipedia.org','youtube.com','it.wikipedia.org']

    poison = ['youtube','wikipedia']
    dirty = [word for word in deduplicated if any(unwanted in word for unwanted in poison)]
    clean = [word for word in deduplicated if word not in dirty]

    print(dirty) # => ['en.wikipedia.org', 'youtube.com', 'it.wikipedia.org']
    print(clean) # => ['facebook.com', 'google.com']