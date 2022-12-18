import os


def get_best_model(base_dir):
    last_modified_time = 0.0
    last_modified_path = ""

    for dirname in os.listdir(base_dir):
        path = os.path.join(base_dir, dirname)
        modified_time = os.path.getmtime(path)
        if modified_time > last_modified_time:
            last_modified_time = modified_time
            last_modified_path = path
    return last_modified_path

